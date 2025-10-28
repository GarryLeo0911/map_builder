#include "map_builder/point_cloud_processor.hpp"
#include <chrono>

namespace map_builder
{

PointCloudProcessor::PointCloudProcessor()
    : Node("point_cloud_processor")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Point Cloud Processor");

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and get parameters
    declareParameters();
    getParameters();

    // Initialize subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "oak/points", 10,
        std::bind(&PointCloudProcessor::pointcloudCallback, this, std::placeholders::_1));

    // Initialize publishers
    filtered_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_builder/filtered_points", 10);
    
    accumulated_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_builder/accumulated_points", 10);

    // Initialize timer for publishing accumulated point cloud
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&PointCloudProcessor::publishAccumulatedPointCloud, this));

    RCLCPP_INFO(this->get_logger(), "Point Cloud Processor initialized");
}

void PointCloudProcessor::declareParameters()
{
    this->declare_parameter<double>("voxel_size", 0.05);
    this->declare_parameter<double>("max_range", 10.0);
    this->declare_parameter<double>("min_range", 0.3);
    this->declare_parameter<int>("statistical_outlier_nb_neighbors", 20);
    this->declare_parameter<double>("statistical_outlier_std_ratio", 2.0);
    this->declare_parameter<int>("buffer_size", 100);
}

void PointCloudProcessor::getParameters()
{
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("statistical_outlier_nb_neighbors", statistical_outlier_nb_neighbors_);
    this->get_parameter("statistical_outlier_std_ratio", statistical_outlier_std_ratio_);
    this->get_parameter("buffer_size", buffer_size_);

    RCLCPP_INFO(this->get_logger(), "Parameters loaded - voxel_size: %.3f, range: [%.2f, %.2f]", 
                voxel_size_, min_range_, max_range_);
}

void PointCloudProcessor::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        // Filter the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterPointCloud(cloud);

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Point cloud empty after filtering");
            return;
        }

        // Store in buffer
        {
            std::lock_guard<std::mutex> lock(processing_mutex_);
            pointcloud_buffer_.push_back(filtered_cloud);
            if (static_cast<int>(pointcloud_buffer_.size()) > buffer_size_)
            {
                pointcloud_buffer_.pop_front();
            }
        }

        // Publish filtered point cloud
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_pointcloud_pub_->publish(filtered_msg);

        RCLCPP_DEBUG(this->get_logger(), "Processed point cloud: %zu -> %zu points", 
                     cloud->size(), filtered_cloud->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = cloud;

    // Range filtering
    filtered_cloud = rangeFilter(filtered_cloud);

    // Voxel downsampling
    filtered_cloud = voxelDownsample(filtered_cloud);

    // Statistical outlier removal
    if (filtered_cloud->size() > static_cast<size_t>(statistical_outlier_nb_neighbors_))
    {
        filtered_cloud = statisticalOutlierRemoval(filtered_cloud);
    }

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::rangeFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : cloud->points)
    {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        
        if (distance >= min_range_ && distance <= max_range_ && 
            std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
        {
            filtered_cloud->points.push_back(point);
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::voxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*downsampled_cloud);

    return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud);
    outlier_filter.setMeanK(statistical_outlier_nb_neighbors_);
    outlier_filter.setStddevMulThresh(statistical_outlier_std_ratio_);
    outlier_filter.filter(*filtered_cloud);

    return filtered_cloud;
}

void PointCloudProcessor::publishAccumulatedPointCloud()
{
    try
    {
        std::lock_guard<std::mutex> lock(processing_mutex_);

        if (pointcloud_buffer_.empty())
        {
            return;
        }

        // Combine all point clouds from buffer
        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& cloud : pointcloud_buffer_)
        {
            *accumulated_cloud += *cloud;
        }

        if (accumulated_cloud->empty())
        {
            return;
        }

        // Apply additional downsampling to combined cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_accumulated = voxelDownsample(accumulated_cloud);

        // Publish accumulated point cloud
        sensor_msgs::msg::PointCloud2 accumulated_msg;
        pcl::toROSMsg(*downsampled_accumulated, accumulated_msg);
        accumulated_msg.header.stamp = this->get_clock()->now();
        accumulated_msg.header.frame_id = "oak_camera_frame";
        accumulated_pointcloud_pub_->publish(accumulated_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published accumulated point cloud with %zu points", 
                     downsampled_accumulated->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error publishing accumulated point cloud: %s", e.what());
    }
}

} // namespace map_builder