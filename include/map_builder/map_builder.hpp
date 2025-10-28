#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <mutex>
#include <memory>
#include <chrono>

namespace map_builder
{

// Structure to hold keyframe data for loop closure detection
struct KeyFrame
{
    int id;
    rclcpp::Time timestamp;
    geometry_msgs::msg::Pose pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features;
};

class MapBuilder : public rclcpp::Node
{
public:
    // Point cloud processing
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;
    
    MapBuilder();
    ~MapBuilder() = default;

private:
    // Basic ROS2 parameters
    double map_resolution_;
    int map_width_;
    int map_height_;
    double map_origin_x_;
    double map_origin_y_;
    double robot_radius_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    
    // Loop closure detection parameters
    bool enable_loop_closure_;
    double loop_closure_distance_threshold_;
    double loop_closure_feature_threshold_;
    int min_loop_closure_interval_;
    int keyframe_buffer_size_;
    
    // Memory management parameters
    bool enable_memory_management_;
    int max_map_points_;
    double memory_cleanup_interval_;
    
    // Map update parameters
    double occupancy_hit_probability_;
    double occupancy_miss_probability_;
    double occupancy_min_probability_;
    double occupancy_max_probability_;

    // Enhanced map data
    std::vector<int8_t> occupancy_map_;
    std::vector<float> height_map_;
    std::vector<double> probability_map_;  // Probabilistic occupancy tracking
    std::mutex map_mutex_;

    // Loop closure data
    std::deque<KeyFrame> keyframes_;
    std::mutex keyframes_mutex_;
    int frame_counter_;

    // Current robot pose
    geometry_msgs::msg::PoseStamped::SharedPtr current_robot_pose_;

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // Timer for publishing occupancy grid
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Callback functions
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishOccupancyGrid();

    // Enhanced map processing functions
    void updateEnhancedOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
    void enhancedRayTraceFreespace(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
    double updateProbability(double current_prob, double sensor_prob);
    
    // Legacy map processing functions (kept for compatibility)
    void updateOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
    void rayTraceFreespace(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
    std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1);
    bool isValidGridCoordinate(int x, int y) const;
    int gridIndex(int x, int y) const;

    // Loop closure detection functions
    void storeKeyframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, rclcpp::Time timestamp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints);
    void detectLoopClosures();
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    double calculateFeatureSimilarity(pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2);
    bool verifyGeometricConsistency(const KeyFrame& current, const KeyFrame& candidate);
    void applyLoopClosureCorrection(const KeyFrame& current, const KeyFrame& candidate);

    // Memory management functions
    void manageMemory();
    void performMemoryCleanup();

    // Transform functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointsToMapFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr points, const std_msgs::msg::Header& header);

    // Utility functions
    void declareParameters();
    void getParameters();
    void initializeMap();
};

} // namespace map_builder