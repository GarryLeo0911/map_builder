#ifndef MAP_BUILDER__ENHANCED_VISUAL_ODOMETRY_HPP_
#define MAP_BUILDER__ENHANCED_VISUAL_ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
// Note: cv_bridge headers will be included through ROS2 dependencies

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Dense>
#include <deque>
#include <mutex>

namespace map_builder
{

struct VisualFrame
{
    rclcpp::Time timestamp;
    cv::Mat rgb_image;
    cv::Mat depth_image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    geometry_msgs::msg::Pose pose;
    sensor_msgs::msg::Imu imu_data;
    bool has_imu;
    int frame_id;
};

class EnhancedVisualOdometry : public rclcpp::Node
{
public:
    EnhancedVisualOdometry();

private:
    // ROS2 communication
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    
    rclcpp::TimerBase::SharedPtr odometry_timer_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Parameters
    void declareParameters();
    void getParameters();
    
    // Core algorithm parameters
    double feature_detector_threshold_;
    int max_features_;
    double match_ratio_threshold_;
    int min_matches_;
    double ransac_threshold_;
    double max_translation_per_frame_;
    double max_rotation_per_frame_;
    
    // IMU fusion parameters
    bool enable_imu_fusion_;
    double imu_weight_;
    double gravity_magnitude_;
    
    // Point cloud odometry parameters
    double voxel_size_;
    double icp_max_correspondence_distance_;
    int icp_max_iterations_;
    double icp_transformation_epsilon_;
    
    // Frame management
    std::deque<VisualFrame> frame_buffer_;
    int max_frame_buffer_size_;
    int current_frame_id_;
    
    // Feature detection and matching
    cv::Ptr<cv::ORB> feature_detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    
    // State
    std::mutex odometry_mutex_;
    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::Odometry current_odometry_;
    Eigen::Matrix4d cumulative_transform_;
    bool odometry_initialized_;
    
    // IMU integration
    sensor_msgs::msg::Imu last_imu_data_;
    bool imu_initialized_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d bias_gyro_;
    Eigen::Vector3d bias_accel_;
    
    // Callback functions
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // Core processing functions
    void processVisualOdometry();
    VisualFrame processFrame(const cv::Mat& rgb_image, const cv::Mat& depth_image, 
                           pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                           const rclcpp::Time& timestamp);
    
    // Feature-based visual odometry
    std::vector<cv::KeyPoint> detectFeatures(const cv::Mat& image);
    cv::Mat extractDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints);
    std::vector<cv::DMatch> matchFeatures(const cv::Mat& desc1, const cv::Mat& desc2);
    Eigen::Matrix4d estimateMotionFromFeatures(const VisualFrame& frame1, const VisualFrame& frame2);
    
    // Point cloud-based odometry (fallback)
    Eigen::Matrix4d estimateMotionFromPointClouds(const VisualFrame& frame1, const VisualFrame& frame2);
    
    // IMU integration
    void integrateIMU(const sensor_msgs::msg::Imu& imu_data, double dt);
    Eigen::Matrix4d predictMotionFromIMU(double dt);
    void fuseVisualIMU(const Eigen::Matrix4d& visual_motion, const Eigen::Matrix4d& imu_motion, double dt);
    
    // Utility functions
    bool validateTransformation(const Eigen::Matrix4d& transform);
    void updateOdometry(const Eigen::Matrix4d& transform, const rclcpp::Time& timestamp);
    void publishOdometry();
    void broadcastTransform(const rclcpp::Time& timestamp);
    
    // Conversion utilities
    geometry_msgs::msg::Pose poseFromMatrix4d(const Eigen::Matrix4d& matrix);
    Eigen::Matrix4d matrix4dFromPose(const geometry_msgs::msg::Pose& pose);
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Temporary storage for synchronized data
    cv::Mat latest_rgb_image_;
    cv::Mat latest_depth_image_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_point_cloud_;
    rclcpp::Time latest_data_timestamp_;
    bool has_rgb_data_;
    bool has_depth_data_;
    bool has_pointcloud_data_;
};

} // namespace map_builder

#endif // MAP_BUILDER__ENHANCED_VISUAL_ODOMETRY_HPP_