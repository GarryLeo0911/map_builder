#!/usr/bin/env python3

import sys
import time
import os
import struct
import traceback
import numpy as np
import pcl

import rclpy
from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2

from map_builder.srv import MeshFromPointCloud2


# Database of small test mesh PLY files
# https://people.sc.fsu.edu/~jburkardt/data/ply/ply.html


def pcl_to_ros(pcl_array, frame_id="world"):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:test_mesh_service.py
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    ros_msg.header.frame_id = frame_id

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
        name="x",
        offset=0,
        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
        name="y",
        offset=4,
        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
        name="z",
        offset=8,
        datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
        name="rgb",
        offset=16,
        datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, 0, 0, 0, 0, 0, 0, 0))

    ros_msg.data = b''.join(buffer)

    return ros_msg


def run_test(folder, node: Node):
    node.get_logger().info('Starting batch reconstruction test')

    ply_file_paths = []
    for file in os.listdir(folder):
        if file.endswith('.ply'):
            ply_file_paths.append(os.path.join(folder, file))
            print(len(ply_file_paths), ply_file_paths[-1])

    if len(ply_file_paths) > 2:
        # match original behaviour removing first two entries if present
        ply_file_paths = ply_file_paths[2:]

    client = node.create_client(MeshFromPointCloud2, '/mesh_from_pointclouds')

    for f_path in ply_file_paths:
        if not client.wait_for_service(timeout_sec=3.0):
            node.get_logger().error('mesh_from_pointclouds service not available')
            return

        print('loading cloud...', f_path)
        pcl_cloud = pcl.load(f_path)
        print('cloud shape:', pcl_cloud.size)

        cloud = pcl.PointCloud(np.array(pcl_cloud, dtype=np.float32))
        filtered_msg = pcl_to_ros(cloud)

        mesh_src_point = Point(0, 0, 0)

        for i in range(3):
            try:
                req = MeshFromPointCloud2.Request()
                req.input = filtered_msg
                req.src = mesh_src_point
                time1 = time.time()
                print('mesh_from_pointcloud...')
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future)
                resp1 = future.result()
                time2 = time.time()

                node.get_logger().info('pointcloud processed result: %s' % str(resp1))
                node.get_logger().info('service executed in %f seconds' % (time2 - time1))
            except Exception as e:
                node.get_logger().error('Exception: %s' % str(e))
                traceback.print_exc()

    # os.system("rosnode kill read_stl_node");
    # rospy.signal_shutdown(0)
    # sys.exit()


if __name__ == '__main__':
    rclpy.init()
    node = Node('test_load_stl_call_map_builder_service')
    node.get_logger().info('Initializing test...')
    time.sleep(2)
    try:
        run_test('/media/h3ct0r/f59508e7-022c-4055-bdac-217324eaf1af/home/h3ct0r/Desktop/bag_ufmg_multi_level_22_12_2020/pclouds/', node)
    finally:
        node.destroy_node()
        rclpy.shutdown()