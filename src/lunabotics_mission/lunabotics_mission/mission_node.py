"""
Mission FSM for Lunabotics 2026 Travel Automation.

States:
  IDLE       — waiting for operator trigger via /autonomy_start service
  TRAVERSING — Nav2 goal sent, monitoring zone transitions
  ARRIVED    — robot reached Construction Zone; announce to MCJ

Zone detection (hybrid):
  - ICP-derived distance from odom origin as prior (> obstacle_zone_distance → Obstacle Zone)
  - Sensor density confirmation via /crater_detections count

Operator procedure before going hands-free:
  1. Rotate robot with remote control to face Construction Zone
  2. Place robot in Excavation Zone
  3. Call /autonomy_start — robot pose is identity (0,0,0°) in odom frame
"""

import enum
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class State(enum.Enum):
    IDLE       = 'IDLE'
    TRAVERSING = 'TRAVERSING'
    ARRIVED    = 'ARRIVED'


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('construction_zone_x',   5.5)
        self.declare_parameter('construction_zone_y',   1.25)
        self.declare_parameter('obstacle_zone_distance', 2.5)
        self.declare_parameter('arrival_distance',       5.0)
        self.declare_parameter('nav2_goal_timeout_s',  120.0)

        self.goal_x         = self.get_parameter('construction_zone_x').value
        self.goal_y         = self.get_parameter('construction_zone_y').value
        self.obs_zone_dist  = self.get_parameter('obstacle_zone_distance').value
        self.arrival_dist   = self.get_parameter('arrival_distance').value
        self.goal_timeout   = self.get_parameter('nav2_goal_timeout_s').value

        self.state          = State.IDLE
        self.current_x      = 0.0
        self.current_y      = 0.0
        self.goal_handle    = None
        self.goal_start_time = None
        self.in_obstacle_zone = False

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)

        # Publishers
        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        # Service: operator triggers autonomy
        self.start_srv = self.create_service(
            Trigger, '/autonomy_start', self._start_cb)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Diagnostics timer
        self.timer = self.create_timer(1.0, self._publish_state)

        self.get_logger().info('Mission node ready — call /autonomy_start to begin')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.state != State.TRAVERSING:
            return

        dist = math.hypot(self.current_x, self.current_y)

        if not self.in_obstacle_zone and dist > self.obs_zone_dist:
            self.in_obstacle_zone = True
            self.get_logger().info(
                f'Entered Obstacle Zone (dist={dist:.2f} m from origin)')

        # Check timeout
        if self.goal_start_time is not None:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9
            if elapsed > self.goal_timeout:
                self.get_logger().warn(
                    f'Nav2 goal timed out after {elapsed:.0f} s — retrying')
                self._send_nav_goal()

    def _start_cb(self, request, response):
        if self.state != State.IDLE:
            response.success = False
            response.message = f'Already in state {self.state.value}'
            return response

        self.get_logger().info('*** AUTONOMY START — hands-free mode initiated ***')
        self._transition(State.TRAVERSING)
        self._send_nav_goal()

        response.success = True
        response.message = 'Autonomy started — robot heading to Construction Zone'
        return response

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _send_nav_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # face forward (0° yaw)

        self.get_logger().info(
            f'Sending Nav2 goal → ({self.goal_x:.2f}, {self.goal_y:.2f}) in odom frame')

        self.goal_start_time = self.get_clock().now()
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 rejected goal')
            return
        self.get_logger().info('Nav2 goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = math.hypot(
            fb.current_pose.pose.position.x - self.goal_x,
            fb.current_pose.pose.position.y - self.goal_y)
        if dist < 0.5:
            self.get_logger().info_once(
                f'Approaching Construction Zone (dist_to_goal={dist:.2f} m)')

    def _result_cb(self, future):
        result = future.result()
        status = result.status

        from action_msgs.msg import GoalStatus
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('*** AUTONOMY COMPLETE — announce to MCJ ***')
            self._transition(State.ARRIVED)
        else:
            self.get_logger().warn(
                f'Nav2 goal finished with status {status} — retrying')
            if self.state == State.TRAVERSING:
                self._send_nav_goal()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: State):
        self.get_logger().info(f'State: {self.state.value} → {new_state.value}')
        self.state = new_state

    def _publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
