#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <deque>
#include <mutex>
#include <memory>

namespace map_builder
{

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor();
    ~PointCloudProcessor() = default;

private:
    // ROS2 parameters
    double voxel_size_;
    double max_range_;
    double min_range_;
    int statistical_outlier_nb_neighbors_;
    double statistical_outlier_std_ratio_;
    int buffer_size_;

    // Point cloud processing
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;
    
    std::deque<PointCloud::Ptr> pointcloud_buffer_;
    std::mutex processing_mutex_;

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_pointcloud_pub_;

    // Timer for publishing accumulated point cloud
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Callback functions
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishAccumulatedPointCloud();

    // Processing functions
    PointCloud::Ptr filterPointCloud(PointCloud::Ptr cloud);
    PointCloud::Ptr voxelDownsample(PointCloud::Ptr cloud);
    PointCloud::Ptr rangeFilter(PointCloud::Ptr cloud);
    PointCloud::Ptr statisticalOutlierRemoval(PointCloud::Ptr cloud);

    // Utility functions
    void declareParameters();
    void getParameters();
};

} // namespace map_builder