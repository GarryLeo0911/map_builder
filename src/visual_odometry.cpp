#include "map_builder/visual_odometry.hpp"
#include <chrono>
#include <tf2_eigen/tf2_eigen.h>

namespace map_builder
{

VisualOdometry::VisualOdometry()
    : Node("visual_odometry"),
      cumulative_transform_(Eigen::Matrix4f::Identity()),
      pose_initialized_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Visual Odometry");

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Declare and get parameters
    declareParameters();
    getParameters();

    // Initialize subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "oak/points", 10,
        std::bind(&VisualOdometry::pointcloudCallback, this, std::placeholders::_1));

    // Initialize publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "visual_odometry/pose", 10);

    // Initialize timer for publishing pose
    pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz
        std::bind(&VisualOdometry::publishPose, this));

    // Initialize pose
    initializePose();

    RCLCPP_INFO(this->get_logger(), "Visual Odometry initialized");
}

void VisualOdometry::declareParameters()
{
    this->declare_parameter<double>("voxel_size", 0.02);
    this->declare_parameter<double>("max_correspondence_distance", 0.05);
    this->declare_parameter<int>("max_iterations", 50);
    this->declare_parameter<double>("transformation_epsilon", 1e-6);
    this->declare_parameter<double>("euclidean_fitness_epsilon", 1e-6);
    this->declare_parameter<double>("max_translation", 0.5);
    this->declare_parameter<double>("max_rotation", 0.5);
    this->declare_parameter<bool>("enable_normal_estimation", false);
    this->declare_parameter<double>("normal_search_radius", 0.03);
    this->declare_parameter<bool>("use_gicp", true);
    this->declare_parameter<double>("fitness_score_threshold", 0.3);
}

void VisualOdometry::getParameters()
{
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("max_correspondence_distance", max_correspondence_distance_);
    this->get_parameter("max_iterations", max_iterations_);
    this->get_parameter("transformation_epsilon", transformation_epsilon_);
    this->get_parameter("euclidean_fitness_epsilon", euclidean_fitness_epsilon_);
    this->get_parameter("max_translation", max_translation_);
    this->get_parameter("max_rotation", max_rotation_);
    this->get_parameter("enable_normal_estimation", enable_normal_estimation_);
    this->get_parameter("normal_search_radius", normal_search_radius_);
    this->get_parameter("use_gicp", use_gicp_);
    this->get_parameter("fitness_score_threshold", fitness_score_threshold_);

    RCLCPP_INFO(this->get_logger(), "Visual Odometry parameters loaded - voxel_size: %.3f, max_corr_dist: %.3f, use_gicp: %s", 
                voxel_size_, max_correspondence_distance_, use_gicp_ ? "true" : "false");
}

void VisualOdometry::initializePose()
{
    current_pose_.header.frame_id = "map";
    current_pose_.pose.position.x = 0.0;
    current_pose_.pose.position.y = 0.0;
    current_pose_.pose.position.z = 0.0;
    current_pose_.pose.orientation.x = 0.0;
    current_pose_.pose.orientation.y = 0.0;
    current_pose_.pose.orientation.z = 0.0;
    current_pose_.pose.orientation.w = 1.0;
}

void VisualOdometry::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *current_cloud);

        if (current_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        // Preprocess the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = preprocessPointCloud(current_cloud);

        if (processed_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Point cloud empty after preprocessing");
            return;
        }

        std::lock_guard<std::mutex> lock(odometry_mutex_);

        if (!previous_cloud_ || previous_cloud_->empty())
        {
            // First frame - initialize
            previous_cloud_ = processed_cloud;
            reference_cloud_ = processed_cloud;
            pose_initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Initialized visual odometry with %zu points", processed_cloud->size());
            return;
        }

        // Estimate motion
        Eigen::Matrix4f transformation = estimateMotion(processed_cloud, previous_cloud_);

        if (validateTransformation(transformation))
        {
            // Update cumulative transformation and pose
            updatePose(transformation, msg->header);
            
            // Broadcast transform
            broadcastTransform(msg->header);
            
            // Update previous cloud
            previous_cloud_ = processed_cloud;
            
            RCLCPP_DEBUG(this->get_logger(), "Updated pose - x: %.3f, y: %.3f, z: %.3f", 
                         current_pose_.pose.position.x, 
                         current_pose_.pose.position.y, 
                         current_pose_.pose.position.z);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid transformation detected, skipping update");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in visual odometry: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisualOdometry::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        // Remove NaN points and apply range filter
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);

        // Range filtering (remove very close and very far points)
        pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : filtered_cloud->points)
        {
            double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance > 0.3 && distance < 8.0 && 
                std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
            {
                range_filtered->points.push_back(point);
            }
        }
        range_filtered->width = range_filtered->points.size();
        range_filtered->height = 1;
        range_filtered->is_dense = true;

        // Voxel downsampling
        if (!range_filtered->empty() && voxel_size_ > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(range_filtered);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*filtered_cloud);
        }
        else
        {
            filtered_cloud = range_filtered;
        }

        RCLCPP_DEBUG(this->get_logger(), "Preprocessed point cloud: %zu -> %zu points", 
                     cloud->size(), filtered_cloud->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error preprocessing point cloud: %s", e.what());
        filtered_cloud = cloud;
    }

    return filtered_cloud;
}

Eigen::Matrix4f VisualOdometry::estimateMotion(pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud)
{
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    try
    {
        if (current_cloud->size() < 100 || previous_cloud->size() < 100)
        {
            RCLCPP_WARN(this->get_logger(), "Insufficient points for registration (%zu, %zu)", 
                        current_cloud->size(), previous_cloud->size());
            return transformation;
        }

        if (use_gicp_)
        {
            // Use Generalized ICP for better handling of structured point clouds
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
            gicp.setInputSource(current_cloud);
            gicp.setInputTarget(previous_cloud);
            gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);
            gicp.setMaximumIterations(max_iterations_);
            gicp.setTransformationEpsilon(transformation_epsilon_);
            gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

            pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
            gicp.align(aligned_cloud);

            if (gicp.hasConverged())
            {
                double fitness_score = gicp.getFitnessScore();
                RCLCPP_DEBUG(this->get_logger(), "GICP converged with fitness score: %f", fitness_score);
                
                if (fitness_score < fitness_score_threshold_)
                {
                    transformation = gicp.getFinalTransformation();
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "GICP fitness score too high: %f", fitness_score);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "GICP did not converge");
            }
        }
        else
        {
            // Use standard ICP
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(current_cloud);
            icp.setInputTarget(previous_cloud);
            icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
            icp.setMaximumIterations(max_iterations_);
            icp.setTransformationEpsilon(transformation_epsilon_);
            icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

            pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
            icp.align(aligned_cloud);

            if (icp.hasConverged())
            {
                double fitness_score = icp.getFitnessScore();
                RCLCPP_DEBUG(this->get_logger(), "ICP converged with fitness score: %f", fitness_score);
                
                if (fitness_score < fitness_score_threshold_)
                {
                    transformation = icp.getFinalTransformation();
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "ICP did not converge");
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in motion estimation: %s", e.what());
    }

    return transformation;
}

bool VisualOdometry::validateTransformation(const Eigen::Matrix4f& transform)
{
    try
    {
        // Extract translation and rotation
        Eigen::Vector3f translation = transform.block<3, 1>(0, 3);
        Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);

        // Check translation magnitude
        double translation_norm = translation.norm();
        if (translation_norm > max_translation_)
        {
            RCLCPP_WARN(this->get_logger(), "Large translation detected: %.3f > %.3f", 
                        translation_norm, max_translation_);
            return false;
        }

        // Check rotation magnitude (convert to angle-axis)
        Eigen::AngleAxisf angle_axis(rotation);
        double rotation_angle = std::abs(angle_axis.angle());
        if (rotation_angle > max_rotation_)
        {
            RCLCPP_WARN(this->get_logger(), "Large rotation detected: %.3f > %.3f", 
                        rotation_angle, max_rotation_);
            return false;
        }

        // Check for NaN or infinite values
        if (!transform.allFinite())
        {
            RCLCPP_WARN(this->get_logger(), "Transformation contains NaN or infinite values");
            return false;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error validating transformation: %s", e.what());
        return false;
    }
}

void VisualOdometry::updatePose(const Eigen::Matrix4f& transform, const std_msgs::msg::Header& header)
{
    try
    {
        // Apply transformation to cumulative transform
        cumulative_transform_ = cumulative_transform_ * transform;

        // Convert to pose
        current_pose_.pose = poseFromMatrix4f(cumulative_transform_);
        current_pose_.header.stamp = header.stamp;
        current_pose_.header.frame_id = "map";
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error updating pose: %s", e.what());
    }
}

void VisualOdometry::broadcastTransform(const std_msgs::msg::Header& header)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        transform_stamped.header.stamp = header.stamp;
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        
        transform_stamped.transform.translation.x = current_pose_.pose.position.x;
        transform_stamped.transform.translation.y = current_pose_.pose.position.y;
        transform_stamped.transform.translation.z = current_pose_.pose.position.z;
        transform_stamped.transform.rotation = current_pose_.pose.orientation;
        
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error broadcasting transform: %s", e.what());
    }
}

void VisualOdometry::publishPose()
{
    try
    {
        std::lock_guard<std::mutex> lock(odometry_mutex_);
        
        if (pose_initialized_)
        {
            current_pose_.header.stamp = this->get_clock()->now();
            pose_pub_->publish(current_pose_);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error publishing pose: %s", e.what());
    }
}

Eigen::Matrix4f VisualOdometry::matrix4fFromPose(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    // Translation
    matrix(0, 3) = pose.position.x;
    matrix(1, 3) = pose.position.y;
    matrix(2, 3) = pose.position.z;
    
    // Rotation (quaternion to rotation matrix)
    Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, 
                        pose.orientation.y, pose.orientation.z);
    matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    return matrix;
}

geometry_msgs::msg::Pose VisualOdometry::poseFromMatrix4f(const Eigen::Matrix4f& matrix)
{
    geometry_msgs::msg::Pose pose;
    
    // Translation
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    
    // Rotation (rotation matrix to quaternion)
    Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation);
    
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

} // namespace map_builder