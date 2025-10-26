#!/usr/bin/env python3
"""Publish a visualization_msgs/Marker pointing to a mesh file (ROS2).

This uses ament index to find the package share folder.
"""
import time
import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory


class MeshMarkerPublisher(Node):
    def __init__(self):
        super().__init__('test_publish_stl_marker')
        self.pub = self.create_publisher(Marker, '/reconstructed_mesh_marker_normal', 1)
        pkg_path = get_package_share_directory('map_builder')
        self.mesh_path = os.path.join(pkg_path, 'test', 'map_medium_mesh.stl')

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = 'file://' + self.mesh_path
        marker.action = marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MeshMarkerPublisher()
    try:
        while rclpy.ok():
            node.publish_marker()
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()