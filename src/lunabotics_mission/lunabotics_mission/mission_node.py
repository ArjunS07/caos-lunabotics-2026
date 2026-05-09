"""
Mission FSM for Lunabotics 2026 Travel Automation.

States:
  IDLE       — waiting for operator trigger via /autonomy_start service
  TRAVERSING — Nav2 goal sent, monitoring zone transitions
  ARRIVED    — robot reached Construction Zone; announce to MCJ

Operator procedure before going hands-free:
  1. Place robot in Excavation Zone facing Construction Zone
  2. Verify /odometry/filtered shows near-zero pose
  3. Call /autonomy_start — robot pose is identity (0,0,0°) in odom frame
"""

import enum
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

_MAX_RETRIES = 3


class State(enum.Enum):
    IDLE       = 'IDLE'
    TRAVERSING = 'TRAVERSING'
    ARRIVED    = 'ARRIVED'


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('construction_zone_x',    5.5)
        self.declare_parameter('construction_zone_y',    1.25)
        self.declare_parameter('obstacle_zone_distance', 2.5)
        self.declare_parameter('nav2_goal_timeout_s',  120.0)

        self.goal_x        = self.get_parameter('construction_zone_x').value
        self.goal_y        = self.get_parameter('construction_zone_y').value
        self.obs_zone_dist = self.get_parameter('obstacle_zone_distance').value
        self.goal_timeout  = self.get_parameter('nav2_goal_timeout_s').value

        self.state           = State.IDLE
        self.current_x       = 0.0
        self.current_y       = 0.0
        self.goal_handle     = None
        self.goal_start_time = None
        self.retry_count     = 0

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)

        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        self.start_srv = self.create_service(
            Trigger, '/autonomy_start', self._start_cb)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(1.0, self._timer_cb)

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
        if dist > self.obs_zone_dist:
            self.get_logger().info_once(
                f'Entered Obstacle Zone (dist={dist:.2f} m from origin)')

    def _start_cb(self, request, response):
        if self.state != State.IDLE:
            response.success = False
            response.message = f'Already in state {self.state.value}'
            return response

        self.get_logger().info('*** AUTONOMY START — hands-free mode initiated ***')
        self.retry_count = 0
        self._transition(State.TRAVERSING)
        self._send_nav_goal(block=True)

        response.success = True
        response.message = 'Autonomy started — robot heading to Construction Zone'
        return response

    def _timer_cb(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

        if self.state != State.TRAVERSING or self.goal_start_time is None:
            return

        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9
        if elapsed > self.goal_timeout:
            self.get_logger().warn(
                f'Nav2 goal timed out after {elapsed:.0f} s — retrying')
            self._send_nav_goal()

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _send_nav_goal(self, block=False):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        timeout = 5.0 if block else 0.0
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Nav2 action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending Nav2 goal → ({self.goal_x:.2f}, {self.goal_y:.2f}) in odom frame')

        self.goal_start_time = self.get_clock().now()
        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 rejected goal')
            self.goal_handle = None
            return
        self.get_logger().info('Nav2 goal accepted')
        self.goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        if dist < 0.5:
            self.get_logger().info_once(
                f'Approaching Construction Zone (dist_to_goal={dist:.2f} m)')

    def _result_cb(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('*** AUTONOMY COMPLETE — announce to MCJ ***')
            self._transition(State.ARRIVED)
            return

        # Intentional cancellation (timeout retry) — new goal already sent
        if status == GoalStatus.STATUS_CANCELED:
            return

        self.retry_count += 1
        if self.retry_count > _MAX_RETRIES:
            self.get_logger().error(
                f'Nav2 goal failed {self.retry_count} times — stopping autonomy')
            self._transition(State.IDLE)
            self.goal_start_time = None
            return

        self.get_logger().warn(
            f'Nav2 goal finished with status {status} — retrying '
            f'({self.retry_count}/{_MAX_RETRIES})')
        if self.state == State.TRAVERSING:
            self._send_nav_goal()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: State):
        self.get_logger().info(f'State: {self.state.value} → {new_state.value}')
        self.state = new_state


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
