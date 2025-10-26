#!/usr/bin/env python3
"""ROS2 test client that subscribes to a pointcloud topic, does basic
PCL filtering and calls the mesh reconstruction service.
"""
import time
import traceback
import struct
import numpy as np
import pcl

import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point

from map_builder.srv import MeshFromPointCloud2


def pcl_to_ros(pcl_array, frame_id="world"):
    ros_msg = PointCloud2()
    ros_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    ros_msg.header.frame_id = frame_id
    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, 0, 0, 0, 0, 0, 0, 0))

    # In Python3, join requires bytes
    ros_msg.data = b''.join(buffer)

    return ros_msg


class TestMeshService(Node):
    def __init__(self):
        super().__init__('test_load_stl_call_map_builder_service')
        self.create_subscription(sensor_msgs.PointCloud2, '/test_point_cloud', self.pointcloud2_callback, 10)
        self.client = self.create_client(MeshFromPointCloud2, '/mesh_from_pointclouds')

    def pointcloud2_callback(self, msg):
        self.get_logger().info('pointcloud2_callback called')
        try:
            if not self.client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error('Mesh service not available')
                return

            points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
            cloud = pcl.PointCloud(np.array(points, dtype=np.float32))

            clip_distance = 20
            passthrough = cloud.make_passthrough_filter()
            passthrough.set_filter_field_name('x')
            passthrough.set_filter_limits(-clip_distance, clip_distance)
            cloud_filtered = passthrough.filter()

            passthrough = cloud_filtered.make_passthrough_filter()
            passthrough.set_filter_field_name('y')
            passthrough.set_filter_limits(-clip_distance, clip_distance)
            cloud_filtered = passthrough.filter()

            passthrough = cloud_filtered.make_passthrough_filter()
            passthrough.set_filter_field_name('z')
            passthrough.set_filter_limits(-clip_distance, clip_distance)
            cloud_filtered = passthrough.filter()

            vg = cloud_filtered.make_voxel_grid_filter()
            vg.set_leaf_size(0.01, 0.01, 0.01)
            cloud_filtered = vg.filter()

            filtered_msg = pcl_to_ros(cloud_filtered, frame_id=msg.header.frame_id)

            time1 = time.time()
            req = MeshFromPointCloud2.Request()
            req.input = filtered_msg
            req.src = Point(x=0.0, y=0.0, z=0.0)

            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            time2 = time.time()

            if resp is not None:
                self.get_logger().info('pointcloud processed result: %s' % str(resp))
                self.get_logger().info('service executed in %f seconds' % (time2 - time1))
            else:
                self.get_logger().error('Service call failed')
        except Exception as e:
            self.get_logger().error('Exception: %s' % str(e))
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = TestMeshService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()