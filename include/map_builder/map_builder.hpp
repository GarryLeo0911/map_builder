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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <Eigen/Dense>
#include <vector>
#include <mutex>
#include <memory>

namespace map_builder
{

class MapBuilder : public rclcpp::Node
{
public:
    // Point cloud processing
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;
    
    MapBuilder();
    ~MapBuilder() = default;

private:
    // ROS2 parameters
    double map_resolution_;
    int map_width_;
    int map_height_;
    double map_origin_x_;
    double map_origin_y_;
    double robot_radius_;
    double min_obstacle_height_;
    double max_obstacle_height_;

    // Map data
    std::vector<int8_t> occupancy_map_;
    std::vector<float> height_map_;
    std::mutex map_mutex_;

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

    // Map processing functions
    void updateOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
    void rayTraceFreespace(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
    std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1);
    bool isValidGridCoordinate(int x, int y) const;
    int gridIndex(int x, int y) const;

    // Transform functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointsToMapFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr points, const std_msgs::msg::Header& header);

    // Utility functions
    void declareParameters();
    void getParameters();
    void initializeMap();
};

} // namespace map_builder