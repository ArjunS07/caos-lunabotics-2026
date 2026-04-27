"""
Converts /crater_detections (PoseArray) to /crater_cloud (PointCloud2).
Nav2 obstacle layer subscribes to the PointCloud2 to mark craters in the costmap.
"""

import struct
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2, PointField


class CraterCloudPublisherNode(Node):

    def __init__(self):
        super().__init__('crater_cloud_publisher_node')

        self.sub = self.create_subscription(
            PoseArray, '/crater_detections', self._cb, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/crater_cloud', 10)

    def _cb(self, msg: PoseArray):
        cloud = PointCloud2()
        cloud.header = msg.header

        cloud.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.is_bigendian = False
        cloud.is_dense     = True
        cloud.height       = 1
        cloud.width        = len(msg.poses)
        cloud.row_step     = cloud.point_step * cloud.width

        data = bytearray()
        for pose in msg.poses:
            data += struct.pack('fff',
                                pose.position.x,
                                pose.position.y,
                                pose.position.z)
        cloud.data = bytes(data)

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = CraterCloudPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
