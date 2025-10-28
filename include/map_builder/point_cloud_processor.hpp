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
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/iss_3d.h>
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
    // Basic ROS2 parameters
    double voxel_size_;
    double max_range_;
    double min_range_;
    int statistical_outlier_nb_neighbors_;
    double statistical_outlier_std_ratio_;
    int buffer_size_;
    
    // Enhanced filtering parameters
    bool enable_bilateral_filter_;
    double bilateral_sigma_s_;
    double bilateral_sigma_r_;
    bool enable_ground_removal_;
    double ground_threshold_;
    bool enable_radius_outlier_removal_;
    double radius_search_;
    int min_neighbors_radius_;
    
    // Registration parameters
    bool enable_registration_;
    double registration_max_correspondence_distance_;
    int registration_max_iterations_;
    double registration_transformation_epsilon_;
    double registration_euclidean_fitness_epsilon_;
    
    // Adaptive voxel sizing parameters
    bool adaptive_voxel_size_;
    double min_voxel_size_;
    double max_voxel_size_;
    int target_points_per_cloud_;

    // Point cloud processing
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;
    
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointcloud_buffer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
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

    // Enhanced processing functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rangeFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // New enhanced filtering methods
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // Adaptive voxel downsampling
    double calculateAdaptiveVoxelSize(size_t point_count);
    pcl::PointCloud<pcl::PointXYZ>::Ptr adaptiveVoxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double voxel_size);
    
    // Registration
    pcl::PointCloud<pcl::PointXYZ>::Ptr registerWithPrevious(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud);

    // Utility functions
    void declareParameters();
    void getParameters();
};

} // namespace map_builder