#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Optional imports - will gracefully handle missing dependencies
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    
try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False
    
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

import sensor_msgs_py.point_cloud2 as pc2
from builtin_interfaces.msg import Time
import time


class OAKDNode(Node):
    def __init__(self):
        super().__init__('oakd_node')
        
        # Check for required dependencies
        if not all([CV2_AVAILABLE, DEPTHAI_AVAILABLE, NUMPY_AVAILABLE, CV_BRIDGE_AVAILABLE]):
            self.get_logger().error(
                f"Missing dependencies: "
                f"cv2={CV2_AVAILABLE}, depthai={DEPTHAI_AVAILABLE}, "
                f"numpy={NUMPY_AVAILABLE}, cv_bridge={CV_BRIDGE_AVAILABLE}"
            )
            self.get_logger().error("Install missing dependencies: pip3 install opencv-python depthai numpy")
            return
        
        # Declare parameters
        self.declare_parameter('fps', 30)
        self.declare_parameter('rgb_resolution', '1080p')
        self.declare_parameter('depth_resolution', '720p')
        self.declare_parameter('confidence_threshold', 200)
        self.declare_parameter('lr_check', True)
        self.declare_parameter('subpixel', False)
        self.declare_parameter('extended_disparity', False)
        self.declare_parameter('baseline', 75.0)  # mm
        self.declare_parameter('focal_length', 883.15)  # pixels
        self.declare_parameter('frame_id', 'oakd_frame')
        self.declare_parameter('publish_tf', True)
        
        # Get parameters
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.rgb_resolution = self.get_parameter('rgb_resolution').get_parameter_value().string_value
        self.depth_resolution = self.get_parameter('depth_resolution').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().integer_value
        self.lr_check = self.get_parameter('lr_check').get_parameter_value().bool_value
        self.subpixel = self.get_parameter('subpixel').get_parameter_value().bool_value
        self.extended_disparity = self.get_parameter('extended_disparity').get_parameter_value().bool_value
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        self.focal_length = self.get_parameter('focal_length').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        # Publishers
        self.rgb_pub = self.create_publisher(Image, 'oakd/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'oakd/depth/image_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'oakd/points', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, 'oakd/rgb/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, 'oakd/depth/camera_info', 10)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.setup_camera()
        
        # Timer for publishing camera info and TF
        self.create_timer(1.0 / self.fps, self.process_frames)
        
        self.get_logger().info('OAK-D driver node initialized')

    def setup_camera(self):
        """Setup DepthAI pipeline for OAK-D camera"""
        try:
            # Create pipeline
            self.pipeline = dai.Pipeline()
            
            # Define sources and outputs
            cam_rgb = self.pipeline.create(dai.node.ColorCamera)
            mono_left = self.pipeline.create(dai.node.MonoCamera)
            mono_right = self.pipeline.create(dai.node.MonoCamera)
            stereo = self.pipeline.create(dai.node.StereoDepth)
            
            rgb_out = self.pipeline.create(dai.node.XLinkOut)
            depth_out = self.pipeline.create(dai.node.XLinkOut)
            
            rgb_out.setStreamName("rgb")
            depth_out.setStreamName("depth")
            
            # Properties
            cam_rgb.setPreviewSize(640, 480)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
            cam_rgb.setResolution(getattr(dai.ColorCameraProperties.SensorResolution, f'THE_{self.rgb_resolution.upper()}'))
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            cam_rgb.setFps(self.fps)
            
            # Mono cameras
            mono_left.setResolution(getattr(dai.MonoCameraProperties.SensorResolution, f'THE_{self.depth_resolution.upper()}'))
            mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            mono_right.setResolution(getattr(dai.MonoCameraProperties.SensorResolution, f'THE_{self.depth_resolution.upper()}'))
            mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            
            # Stereo depth
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
            stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            stereo.setLeftRightCheck(self.lr_check)
            stereo.setSubpixel(self.subpixel)
            stereo.setExtendedDisparity(self.extended_disparity)
            stereo.initialConfig.setConfidenceThreshold(self.confidence_threshold)
            
            # Linking
            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)
            
            cam_rgb.preview.link(rgb_out.input)
            stereo.depth.link(depth_out.input)
            
            # Connect to device and start pipeline
            self.device = dai.Device(self.pipeline)
            
            # Output queues
            self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            
            self.get_logger().info('Camera pipeline initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {str(e)}')
            raise

    def process_frames(self):
        """Process camera frames and publish ROS messages"""
        try:
            # Get RGB frame
            rgb_frame = self.rgb_queue.get()
            if rgb_frame is not None:
                rgb_cv = rgb_frame.getCvFrame()
                self.publish_rgb_image(rgb_cv)
            
            # Get depth frame
            depth_frame = self.depth_queue.get()
            if depth_frame is not None:
                depth_cv = depth_frame.getFrame()
                self.publish_depth_image(depth_cv)
                self.publish_pointcloud(depth_cv)
            
            # Publish camera info
            self.publish_camera_info()
            
            # Publish TF
            if self.publish_tf:
                self.publish_transform()
                
        except Exception as e:
            self.get_logger().error(f'Error processing frames: {str(e)}')

    def publish_rgb_image(self, cv_image):
        """Publish RGB image"""
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.rgb_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing RGB image: {str(e)}')

    def publish_depth_image(self, depth_image):
        """Publish depth image"""
        try:
            # Convert to 32-bit float in meters
            depth_float = depth_image.astype(np.float32) / 1000.0
            msg = self.bridge.cv2_to_imgmsg(depth_float, '32FC1')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.depth_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing depth image: {str(e)}')

    def publish_pointcloud(self, depth_image):
        """Generate and publish point cloud from depth image"""
        try:
            height, width = depth_image.shape
            
            # Camera intrinsics (approximate for OAK-D)
            fx = self.focal_length
            fy = self.focal_length
            cx = width / 2.0
            cy = height / 2.0
            
            # Generate point cloud
            points = []
            
            # Subsample for performance (every 4th pixel)
            step = 4
            for v in range(0, height, step):
                for u in range(0, width, step):
                    z = depth_image[v, u] / 1000.0  # Convert to meters
                    
                    if z > 0.3 and z < 10.0:  # Filter invalid depths
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        points.append([x, y, z])
            
            if points:
                # Create PointCloud2 message
                header = self.create_header()
                pointcloud_msg = pc2.create_cloud_xyz32(header, points)
                self.pointcloud_pub.publish(pointcloud_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing point cloud: {str(e)}')

    def publish_camera_info(self):
        """Publish camera info for RGB and depth cameras"""
        try:
            # RGB camera info
            rgb_info = CameraInfo()
            rgb_info.header = self.create_header()
            rgb_info.width = 640
            rgb_info.height = 480
            rgb_info.distortion_model = 'plumb_bob'
            rgb_info.k = [self.focal_length, 0.0, 320.0,
                         0.0, self.focal_length, 240.0,
                         0.0, 0.0, 1.0]
            rgb_info.r = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
            rgb_info.p = [self.focal_length, 0.0, 320.0, 0.0,
                         0.0, self.focal_length, 240.0, 0.0,
                         0.0, 0.0, 1.0, 0.0]
            
            self.rgb_info_pub.publish(rgb_info)
            
            # Depth camera info (same as RGB for now)
            depth_info = rgb_info
            depth_info.header = self.create_header()
            self.depth_info_pub.publish(depth_info)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing camera info: {str(e)}')

    def publish_transform(self):
        """Publish TF transform for camera frame"""
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = self.frame_id
            
            # Set transform (camera mounted facing forward)
            t.transform.translation.x = 0.1  # 10cm forward from base_link
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.2  # 20cm up from base_link
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing transform: {str(e)}')

    def create_header(self):
        """Create a standard header with current timestamp"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def destroy_node(self):
        """Clean up resources"""
        try:
            if hasattr(self, 'device'):
                self.device.close()
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OAKDNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()