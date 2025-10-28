#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <mutex>
#include <memory>

namespace map_builder
{

class VisualOdometry : public rclcpp::Node
{
public:
    VisualOdometry();
    ~VisualOdometry() = default;

private:
    // Parameters
    double voxel_size_;
    double max_correspondence_distance_;
    int max_iterations_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    double max_translation_;
    double max_rotation_;
    bool enable_normal_estimation_;
    double normal_search_radius_;
    bool use_gicp_;
    double fitness_score_threshold_;
    
    // Point cloud processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_;
    Eigen::Matrix4f cumulative_transform_;
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Timer for publishing pose
    rclcpp::TimerBase::SharedPtr pose_timer_;
    
    // Mutex for thread safety
    std::mutex odometry_mutex_;
    
    // Current pose
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_initialized_;
    
    // Callback functions
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishPose();
    
    // Odometry functions
    Eigen::Matrix4f estimateMotion(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    bool validateTransformation(const Eigen::Matrix4f& transform);
    
    void updatePose(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header);
    
    void broadcastTransform(const std_msgs::msg::Header& header);
    
    // Utility functions
    void declareParameters();
    void getParameters();
    void initializePose();
    
    Eigen::Matrix4f matrix4fFromPose(const geometry_msgs::msg::Pose& pose);
    geometry_msgs::msg::Pose poseFromMatrix4f(const Eigen::Matrix4f& matrix);
};

} // namespace map_builder