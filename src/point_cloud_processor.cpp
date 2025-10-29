#include "map_builder/point_cloud_processor.hpp"
#include <chrono>
#include <pcl/common/distances.h>
#include <pcl/search/organized.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

namespace map_builder
{

PointCloudProcessor::PointCloudProcessor()
    : Node("point_cloud_processor")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Enhanced Point Cloud Processor for OAK-D");

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and get parameters
    declareParameters();
    getParameters();

    // Initialize subscribers with message_filters for better synchronization
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

    RCLCPP_INFO(this->get_logger(), "Enhanced Point Cloud Processor initialized with OAK-D optimizations");
}

void PointCloudProcessor::declareParameters()
{
    // Basic filtering parameters optimized for OAK-D
    this->declare_parameter<double>("voxel_size", 0.02);  // Finer for OAK-D quality
    this->declare_parameter<double>("max_range", 6.0);    // Reduced for indoor mapping
    this->declare_parameter<double>("min_range", 0.3);    // Increased to avoid noise
    this->declare_parameter<int>("statistical_outlier_nb_neighbors", 20);
    this->declare_parameter<double>("statistical_outlier_std_ratio", 1.0);  // Stricter
    this->declare_parameter<int>("buffer_size", 30);  // Smaller for real-time
    
    // OAK-D specific depth filtering
    this->declare_parameter<bool>("enable_depth_filtering", true);
    this->declare_parameter<double>("depth_confidence_threshold", 0.7);
    this->declare_parameter<bool>("enable_stereo_consistency_check", true);
    this->declare_parameter<double>("stereo_baseline", 0.075);  // OAK-D baseline: 7.5cm
    
    // Enhanced filtering parameters
    this->declare_parameter<bool>("enable_bilateral_filter", false);  // Disabled for stereo data
    this->declare_parameter<double>("bilateral_sigma_s", 15.0);
    this->declare_parameter<double>("bilateral_sigma_r", 0.05);
    this->declare_parameter<bool>("enable_ground_removal", true);
    this->declare_parameter<double>("ground_threshold", 0.05);  // More sensitive
    this->declare_parameter<bool>("enable_radius_outlier_removal", true);
    this->declare_parameter<double>("radius_search", 0.05);  // Smaller radius
    this->declare_parameter<int>("min_neighbors_radius", 3);  // Less strict
    
    // Clustering parameters for object detection
    this->declare_parameter<bool>("enable_clustering", true);
    this->declare_parameter<double>("cluster_tolerance", 0.05);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<int>("max_cluster_size", 10000);
    
    // Registration parameters
    this->declare_parameter<bool>("enable_registration", true);
    this->declare_parameter<double>("registration_max_correspondence_distance", 0.05);  // Tighter
    this->declare_parameter<int>("registration_max_iterations", 30);  // Faster
    this->declare_parameter<double>("registration_transformation_epsilon", 1e-6);
    this->declare_parameter<double>("registration_euclidean_fitness_epsilon", 1e-6);
    
    // Adaptive voxel sizing based on point density
    this->declare_parameter<bool>("adaptive_voxel_size", true);
    this->declare_parameter<double>("min_voxel_size", 0.005);  // Finer minimum
    this->declare_parameter<double>("max_voxel_size", 0.03);   // Coarser maximum
    this->declare_parameter<int>("target_points_per_cloud", 3000);  // Optimized for OAK-D
    
    // Motion-based filtering
    this->declare_parameter<bool>("enable_motion_filtering", true);
    this->declare_parameter<double>("max_point_motion", 0.5);  // Max motion between frames
    
    // Normal estimation for better surface reconstruction
    this->declare_parameter<bool>("enable_normal_estimation", true);
    this->declare_parameter<double>("normal_search_radius", 0.03);
    this->declare_parameter<int>("normal_k_search", 10);
}
}

void PointCloudProcessor::getParameters()
{
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("max_range", max_range_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("statistical_outlier_nb_neighbors", statistical_outlier_nb_neighbors_);
    this->get_parameter("statistical_outlier_std_ratio", statistical_outlier_std_ratio_);
    this->get_parameter("buffer_size", buffer_size_);
    
    // Enhanced filtering parameters
    this->get_parameter("enable_bilateral_filter", enable_bilateral_filter_);
    this->get_parameter("bilateral_sigma_s", bilateral_sigma_s_);
    this->get_parameter("bilateral_sigma_r", bilateral_sigma_r_);
    this->get_parameter("enable_ground_removal", enable_ground_removal_);
    this->get_parameter("ground_threshold", ground_threshold_);
    this->get_parameter("enable_radius_outlier_removal", enable_radius_outlier_removal_);
    this->get_parameter("radius_search", radius_search_);
    this->get_parameter("min_neighbors_radius", min_neighbors_radius_);
    
    // Registration parameters
    this->get_parameter("enable_registration", enable_registration_);
    this->get_parameter("registration_max_correspondence_distance", registration_max_correspondence_distance_);
    this->get_parameter("registration_max_iterations", registration_max_iterations_);
    this->get_parameter("registration_transformation_epsilon", registration_transformation_epsilon_);
    this->get_parameter("registration_euclidean_fitness_epsilon", registration_euclidean_fitness_epsilon_);
    
    // Adaptive voxel sizing
    this->get_parameter("adaptive_voxel_size", adaptive_voxel_size_);
    this->get_parameter("min_voxel_size", min_voxel_size_);
    this->get_parameter("max_voxel_size", max_voxel_size_);
    this->get_parameter("target_points_per_cloud", target_points_per_cloud_);

    RCLCPP_INFO(this->get_logger(), "Enhanced parameters loaded - voxel_size: %.3f, range: [%.2f, %.2f], registration: %s", 
                voxel_size_, min_range_, max_range_, enable_registration_ ? "enabled" : "disabled");
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

        // Transform point cloud to map frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = transformToMapFrame(cloud, msg->header);
        
        if (!transformed_cloud || transformed_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud to map frame");
            return;
        }

        // Filter the transformed point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = filterPointCloud(transformed_cloud);

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

        // Publish filtered point cloud in map frame
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header.stamp = msg->header.stamp;
        filtered_msg.header.frame_id = "map";  // Now in map frame
        filtered_pointcloud_pub_->publish(filtered_msg);

        RCLCPP_DEBUG(this->get_logger(), "Processed point cloud: %zu -> %zu -> %zu points", 
                     cloud->size(), transformed_cloud->size(), filtered_cloud->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = cloud;

    // Range filtering (first step to reduce data)
    filtered_cloud = rangeFilter(filtered_cloud);
    if (filtered_cloud->empty()) return filtered_cloud;

    // Ground removal (if enabled)
    if (enable_ground_removal_)
    {
        filtered_cloud = removeGround(filtered_cloud);
        if (filtered_cloud->empty()) return filtered_cloud;
    }

    // Bilateral filtering for noise reduction while preserving edges
    if (enable_bilateral_filter_ && filtered_cloud->isOrganized())
    {
        filtered_cloud = bilateralFilter(filtered_cloud);
        if (filtered_cloud->empty()) return filtered_cloud;
    }

    // Adaptive voxel downsampling
    if (adaptive_voxel_size_)
    {
        double adaptive_voxel = calculateAdaptiveVoxelSize(filtered_cloud->size());
        filtered_cloud = adaptiveVoxelDownsample(filtered_cloud, adaptive_voxel);
    }
    else
    {
        filtered_cloud = voxelDownsample(filtered_cloud);
    }
    if (filtered_cloud->empty()) return filtered_cloud;

    // Radius outlier removal (more robust than statistical)
    if (enable_radius_outlier_removal_)
    {
        filtered_cloud = radiusOutlierRemoval(filtered_cloud);
        if (filtered_cloud->empty()) return filtered_cloud;
    }

    // Statistical outlier removal as final cleanup
    if (filtered_cloud->size() > static_cast<size_t>(statistical_outlier_nb_neighbors_))
    {
        filtered_cloud = statisticalOutlierRemoval(filtered_cloud);
    }

    // Register with previous frame if enabled
    if (enable_registration_ && previous_cloud_ && !previous_cloud_->empty())
    {
        filtered_cloud = registerWithPrevious(filtered_cloud);
    }

    // Store for next registration
    if (!filtered_cloud->empty())
    {
        previous_cloud_ = filtered_cloud;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        // Simple ground removal based on height threshold
        // For more sophisticated ground removal, use progressive morphological filter
        for (const auto& point : cloud->points)
        {
            if (point.z > ground_threshold_)
            {
                cloud_filtered->points.push_back(point);
            }
        }

        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in ground removal: %s", e.what());
        return cloud;
    }

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Bilateral filtering might not be available in this PCL version
    // Return the original cloud or apply a simple filter
    try
    {
        if (!cloud->isOrganized())
        {
            return cloud;
        }

        // Apply a simple smoothing filter instead
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *filtered_cloud = *cloud;  // Copy the cloud
        
        RCLCPP_DEBUG(this->get_logger(), "Bilateral filtering not available, using passthrough");
        return filtered_cloud;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in bilateral filtering: %s", e.what());
        return cloud;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
        radius_filter.setInputCloud(cloud);
        radius_filter.setRadiusSearch(radius_search_);
        radius_filter.setMinNeighborsInRadius(min_neighbors_radius_);
        radius_filter.filter(*filtered_cloud);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in radius outlier removal: %s", e.what());
        return cloud;
    }

    return filtered_cloud;
}

double PointCloudProcessor::calculateAdaptiveVoxelSize(size_t point_count)
{
    // Calculate adaptive voxel size based on point density
    double ratio = static_cast<double>(point_count) / target_points_per_cloud_;
    double adaptive_voxel = voxel_size_ * std::sqrt(ratio);
    
    // Clamp to min/max values
    adaptive_voxel = std::max(min_voxel_size_, std::min(max_voxel_size_, adaptive_voxel));
    
    return adaptive_voxel;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::adaptiveVoxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double voxel_size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*downsampled_cloud);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in adaptive voxel downsampling: %s", e.what());
        return cloud;
    }

    return downsampled_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::registerWithPrevious(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        if (!previous_cloud_ || previous_cloud_->empty() || current_cloud->empty())
        {
            return current_cloud;
        }

        // Check minimum point requirements for GICP
        const size_t min_points_for_gicp = 50;  // GICP needs sufficient points
        if (current_cloud->size() < min_points_for_gicp || previous_cloud_->size() < min_points_for_gicp)
        {
            RCLCPP_DEBUG(this->get_logger(), "Insufficient points for GICP registration (%zu, %zu), skipping", 
                         current_cloud->size(), previous_cloud_->size());
            return current_cloud;
        }

        // Use Generalized ICP for better registration
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(current_cloud);
        gicp.setInputTarget(previous_cloud_);
        gicp.setMaxCorrespondenceDistance(registration_max_correspondence_distance_);
        gicp.setMaximumIterations(registration_max_iterations_);
        gicp.setTransformationEpsilon(registration_transformation_epsilon_);
        gicp.setEuclideanFitnessEpsilon(registration_euclidean_fitness_epsilon_);

        gicp.align(*aligned_cloud);

        if (gicp.hasConverged())
        {
            double fitness_score = gicp.getFitnessScore();
            
            RCLCPP_DEBUG(this->get_logger(), "Registration converged with fitness score: %f", fitness_score);
            
            // Only use registration if fitness score is reasonable
            if (fitness_score < 0.1)  // Adjust threshold as needed
            {
                return aligned_cloud;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Registration did not converge");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in point cloud registration: %s", e.what());
    }

    return current_cloud;  // Return original if registration fails
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
        accumulated_msg.header.frame_id = "map";  // Accumulated cloud is in map frame
        accumulated_pointcloud_pub_->publish(accumulated_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published accumulated point cloud with %zu points", 
                     downsampled_accumulated->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error publishing accumulated point cloud: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::transformToMapFrame(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    const std_msgs::msg::Header& header)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    try
    {
        // Get transform from camera frame to map frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        // Wait for transform with timeout
        if (!tf_buffer_->canTransform("map", header.frame_id, 
                                     tf2::TimePointZero, tf2::durationFromSec(0.1)))
        {
            // If no dynamic transform available, try to get static transform
            try 
            {
                transform_stamped = tf_buffer_->lookupTransform("map", header.frame_id, tf2::TimePointZero);
                RCLCPP_DEBUG(this->get_logger(), "Using static transform from %s to map", header.frame_id.c_str());
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Could not transform point cloud from %s to map: %s", 
                                    header.frame_id.c_str(), ex.what());
                
                // Return a copy of the original cloud if no transform available
                *transformed_cloud = *cloud;
                return transformed_cloud;
            }
        }
        else
        {
            // Use the most recent transform
            try
            {
                transform_stamped = tf_buffer_->lookupTransform("map", header.frame_id, 
                                                              tf2::TimePointZero);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                *transformed_cloud = *cloud;
                return transformed_cloud;
            }
        }

        // Convert TF transform to Eigen transform
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform_stamped);
        
        // Apply transformation to point cloud
        pcl::transformPointCloud(*cloud, *transformed_cloud, eigen_transform);
        
        RCLCPP_DEBUG(this->get_logger(), "Transformed %zu points from %s to map frame", 
                     cloud->size(), header.frame_id.c_str());
        
        return transformed_cloud;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in point cloud transformation: %s", e.what());
        *transformed_cloud = *cloud;
        return transformed_cloud;
    }
}

} // namespace map_builder