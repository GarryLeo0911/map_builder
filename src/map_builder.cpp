#include "map_builder/map_builder.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

// Enhanced includes for loop closure and optimization
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace map_builder
{

MapBuilder::MapBuilder()
    : Node("map_builder_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Map Builder");

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and get parameters
    declareParameters();
    getParameters();

    // Initialize map
    initializeMap();

    // Initialize subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "map_builder/filtered_points", 10,
        std::bind(&MapBuilder::pointcloudCallback, this, std::placeholders::_1));

    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "robot_pose", 10,
        std::bind(&MapBuilder::robotPoseCallback, this, std::placeholders::_1));

    // Initialize publishers
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map_builder/occupancy_grid", 10);

    // Initialize timer for publishing occupancy grid
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&MapBuilder::publishOccupancyGrid, this));

    RCLCPP_INFO(this->get_logger(), "Map Builder initialized with resolution %.3f m, size %dx%d", 
                map_resolution_, map_width_, map_height_);
}

void MapBuilder::declareParameters()
{
    this->declare_parameter<double>("map_resolution", 0.03);
    this->declare_parameter<int>("map_width", 800);
    this->declare_parameter<int>("map_height", 800);
    this->declare_parameter<double>("map_origin_x", -12.0);
    this->declare_parameter<double>("map_origin_y", -12.0);
    this->declare_parameter<double>("robot_radius", 0.25);
    this->declare_parameter<double>("min_obstacle_height", 0.05);
    this->declare_parameter<double>("max_obstacle_height", 2.5);
    
    // Loop closure detection parameters
    this->declare_parameter<bool>("enable_loop_closure", true);
    this->declare_parameter<double>("loop_closure_distance_threshold", 2.0);
    this->declare_parameter<double>("loop_closure_feature_threshold", 0.7);
    this->declare_parameter<int>("min_loop_closure_interval", 10);
    this->declare_parameter<int>("keyframe_buffer_size", 100);
    
    // Memory management parameters
    this->declare_parameter<bool>("enable_memory_management", true);
    this->declare_parameter<int>("max_map_points", 1000000);
    this->declare_parameter<double>("memory_cleanup_interval", 30.0);
    
    // Map update parameters
    this->declare_parameter<double>("occupancy_hit_probability", 0.7);
    this->declare_parameter<double>("occupancy_miss_probability", 0.4);
    this->declare_parameter<double>("occupancy_min_probability", 0.12);
    this->declare_parameter<double>("occupancy_max_probability", 0.97);
}

void MapBuilder::getParameters()
{
    this->get_parameter("map_resolution", map_resolution_);
    this->get_parameter("map_width", map_width_);
    this->get_parameter("map_height", map_height_);
    this->get_parameter("map_origin_x", map_origin_x_);
    this->get_parameter("map_origin_y", map_origin_y_);
    this->get_parameter("robot_radius", robot_radius_);
    this->get_parameter("min_obstacle_height", min_obstacle_height_);
    this->get_parameter("max_obstacle_height", max_obstacle_height_);
    
    // Loop closure detection parameters
    this->get_parameter("enable_loop_closure", enable_loop_closure_);
    this->get_parameter("loop_closure_distance_threshold", loop_closure_distance_threshold_);
    this->get_parameter("loop_closure_feature_threshold", loop_closure_feature_threshold_);
    this->get_parameter("min_loop_closure_interval", min_loop_closure_interval_);
    this->get_parameter("keyframe_buffer_size", keyframe_buffer_size_);
    
    // Memory management parameters
    this->get_parameter("enable_memory_management", enable_memory_management_);
    this->get_parameter("max_map_points", max_map_points_);
    this->get_parameter("memory_cleanup_interval", memory_cleanup_interval_);
    
    // Map update parameters
    this->get_parameter("occupancy_hit_probability", occupancy_hit_probability_);
    this->get_parameter("occupancy_miss_probability", occupancy_miss_probability_);
    this->get_parameter("occupancy_min_probability", occupancy_min_probability_);
    this->get_parameter("occupancy_max_probability", occupancy_max_probability_);
}

void MapBuilder::initializeMap()
{
    // Initialize occupancy map (-1 = unknown, 0 = free, 100 = occupied)
    occupancy_map_.assign(map_width_ * map_height_, -1);
    
    // Initialize height map (NaN = unknown)
    height_map_.assign(map_width_ * map_height_, std::numeric_limits<float>::quiet_NaN());
    
    // Initialize probability map for better occupancy tracking
    probability_map_.assign(map_width_ * map_height_, 0.5);
    
    // Initialize keyframe storage
    keyframes_.clear();
    frame_counter_ = 0;

    RCLCPP_INFO(this->get_logger(), "Enhanced map initialized with %d cells, loop closure: %s", 
                static_cast<int>(occupancy_map_.size()), enable_loop_closure_ ? "enabled" : "disabled");
}

void MapBuilder::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            return;
        }

        // Transform points to map frame if necessary
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points = transformPointsToMapFrame(cloud, msg->header);

        if (!transformed_points || transformed_points->empty())
        {
            return;
        }

        // Store as keyframe for loop closure detection
        if (enable_loop_closure_)
        {
            storeKeyframe(transformed_points, msg->header.stamp);
            
            // Detect loop closures
            if (frame_counter_ % min_loop_closure_interval_ == 0)
            {
                detectLoopClosures();
            }
        }

        // Update occupancy grid with enhanced probabilistic approach
        updateEnhancedOccupancyGrid(transformed_points);

        // Memory management
        if (enable_memory_management_)
        {
            manageMemory();
        }

        frame_counter_++;

        RCLCPP_DEBUG(this->get_logger(), "Updated enhanced map with %zu points, frame %d", 
                     transformed_points->size(), frame_counter_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud for enhanced mapping: %s", e.what());
    }
}

void MapBuilder::robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_robot_pose_ = msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapBuilder::transformPointsToMapFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr points, const std_msgs::msg::Header& /* header */)
{
    try
    {
        // For simplicity, assume points are already in map frame
        // In a real implementation, you would use TF2 to transform
        return points;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error transforming points: %s", e.what());
        return nullptr;
    }
}

void MapBuilder::updateOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
    try
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        for (const auto& point : points->points)
        {
            // Convert world coordinates to grid coordinates
            int grid_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
            int grid_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_);

            if (!isValidGridCoordinate(grid_x, grid_y))
            {
                continue;
            }

            int index = gridIndex(grid_x, grid_y);

            // Update height map (take maximum height)
            if (std::isnan(height_map_[index]) || point.z > height_map_[index])
            {
                height_map_[index] = point.z;
            }

            // Update occupancy based on height
            if (point.z >= min_obstacle_height_ && point.z <= max_obstacle_height_)
            {
                occupancy_map_[index] = 100; // Occupied
            }
            else
            {
                if (occupancy_map_[index] != 100) // Don't override obstacles
                {
                    occupancy_map_[index] = 0; // Free
                }
            }
        }

        // Mark areas between robot and obstacles as free
        if (current_robot_pose_)
        {
            rayTraceFreespace(points);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error updating occupancy grid: %s", e.what());
    }
}

void MapBuilder::rayTraceFreespace(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
    try
    {
        if (!current_robot_pose_)
        {
            return;
        }

        // Convert robot position to grid coordinates
        int robot_gx = static_cast<int>((current_robot_pose_->pose.position.x - map_origin_x_) / map_resolution_);
        int robot_gy = static_cast<int>((current_robot_pose_->pose.position.y - map_origin_y_) / map_resolution_);

        if (!isValidGridCoordinate(robot_gx, robot_gy))
        {
            return;
        }

        // Ray trace to each point (subsample for performance)
        size_t step = std::max(static_cast<size_t>(1), points->size() / 100);
        for (size_t i = 0; i < points->size(); i += step)
        {
            const auto& point = points->points[i];
            
            int point_gx = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
            int point_gy = static_cast<int>((point.y - map_origin_y_) / map_resolution_);

            if (!isValidGridCoordinate(point_gx, point_gy))
            {
                continue;
            }

            // Bresenham's line algorithm for ray tracing
            auto cells = bresenhamLine(robot_gx, robot_gy, point_gx, point_gy);

            // Mark all cells except the endpoint as free
            for (size_t j = 0; j < cells.size() - 1; ++j)
            {
                int cell_x = cells[j].first;
                int cell_y = cells[j].second;

                if (isValidGridCoordinate(cell_x, cell_y))
                {
                    int index = gridIndex(cell_x, cell_y);
                    if (occupancy_map_[index] != 100) // Don't override obstacles
                    {
                        occupancy_map_[index] = 0; // Free
                    }
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in ray tracing: %s", e.what());
    }
}

std::vector<std::pair<int, int>> MapBuilder::bresenhamLine(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int, int>> points;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0, y = y0;
    
    while (true)
    {
        points.emplace_back(x, y);
        
        if (x == x1 && y == y1)
        {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
    
    return points;
}

bool MapBuilder::isValidGridCoordinate(int x, int y) const
{
    return x >= 0 && x < map_width_ && y >= 0 && y < map_height_;
}

int MapBuilder::gridIndex(int x, int y) const
{
    return y * map_width_ + x;
}

void MapBuilder::publishOccupancyGrid()
{
    try
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

        // Header
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "map";

        // Map metadata
        msg->info.resolution = map_resolution_;
        msg->info.width = map_width_;
        msg->info.height = map_height_;
        msg->info.origin.position.x = map_origin_x_;
        msg->info.origin.position.y = map_origin_y_;
        msg->info.origin.position.z = 0.0;
        msg->info.origin.orientation.w = 1.0;

        // Map data
        msg->data = occupancy_map_;

        occupancy_grid_pub_->publish(std::move(msg));

        RCLCPP_DEBUG(this->get_logger(), "Published occupancy grid");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error publishing occupancy grid: %s", e.what());
    }
}

void MapBuilder::storeKeyframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, rclcpp::Time timestamp)
{
    try
    {
        if (!current_robot_pose_)
        {
            return;
        }

        KeyFrame keyframe;
        keyframe.id = frame_counter_;
        keyframe.timestamp = timestamp;
        keyframe.pose = current_robot_pose_->pose;
        
        // Extract keypoints and features for loop closure
        keyframe.keypoints = extractKeypoints(cloud);
        keyframe.features = extractFeatures(cloud, keyframe.keypoints);
        keyframe.cloud = cloud;

        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            keyframes_.push_back(keyframe);
            
            // Limit keyframe buffer size
            if (static_cast<int>(keyframes_.size()) > keyframe_buffer_size_)
            {
                keyframes_.pop_front();
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Stored keyframe %d with %zu keypoints", 
                     keyframe.id, keyframe.keypoints->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error storing keyframe: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapBuilder::extractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        if (cloud->size() < 100)
        {
            return keypoints;
        }

        // Use ISS3D keypoint detector
        pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        iss_detector.setInputCloud(cloud);
        iss_detector.setSearchMethod(tree);
        iss_detector.setSalientRadius(0.2);
        iss_detector.setNonMaxRadius(0.1);
        iss_detector.setThreshold21(0.975);
        iss_detector.setThreshold32(0.975);
        iss_detector.setMinNeighbors(5);

        iss_detector.compute(*keypoints);

        RCLCPP_DEBUG(this->get_logger(), "Extracted %zu keypoints from %zu points", 
                     keypoints->size(), cloud->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error extracting keypoints: %s", e.what());
        
        // Fallback: uniform sampling
        size_t step = std::max(static_cast<size_t>(1), cloud->size() / 50);
        for (size_t i = 0; i < cloud->size(); i += step)
        {
            keypoints->points.push_back(cloud->points[i]);
        }
        keypoints->width = keypoints->points.size();
        keypoints->height = 1;
        keypoints->is_dense = true;
    }

    return keypoints;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr MapBuilder::extractFeatures(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints)
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);

    try
    {
        if (keypoints->empty() || cloud->size() < 10)
        {
            return features;
        }

        // Estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.1);
        ne.compute(*normals);

        // Extract FPFH features
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(keypoints);
        fpfh.setInputNormals(normals);
        fpfh.setSearchSurface(cloud);
        fpfh.setSearchMethod(tree);
        fpfh.setRadiusSearch(0.2);
        fpfh.compute(*features);

        RCLCPP_DEBUG(this->get_logger(), "Extracted %zu FPFH features", features->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error extracting features: %s", e.what());
    }

    return features;
}

void MapBuilder::detectLoopClosures()
{
    try
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        
        if (keyframes_.size() < 2)
        {
            return;
        }

        const auto& current_keyframe = keyframes_.back();
        
        // Find potential loop closure candidates
        for (auto it = keyframes_.begin(); it != keyframes_.end() - min_loop_closure_interval_; ++it)
        {
            const auto& candidate = *it;
            
            // Check distance threshold
            double distance = calculateDistance(current_keyframe.pose.position, candidate.pose.position);
            if (distance > loop_closure_distance_threshold_)
            {
                continue;
            }

            // Check feature similarity
            double similarity = calculateFeatureSimilarity(current_keyframe.features, candidate.features);
            if (similarity < loop_closure_feature_threshold_)
            {
                continue;
            }

            // Verify geometric consistency
            if (verifyGeometricConsistency(current_keyframe, candidate))
            {
                RCLCPP_INFO(this->get_logger(), "Loop closure detected between frames %d and %d (distance: %.2f, similarity: %.3f)", 
                           current_keyframe.id, candidate.id, distance, similarity);
                
                // Apply loop closure correction
                applyLoopClosureCorrection(current_keyframe, candidate);
                break; // Only process one loop closure per detection cycle
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in loop closure detection: %s", e.what());
    }
}

double MapBuilder::calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double MapBuilder::calculateFeatureSimilarity(
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2)
{
    try
    {
        if (features1->empty() || features2->empty())
        {
            return 0.0;
        }

        // Simple feature matching using correspondence estimation
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr_est;
        corr_est.setInputSource(features1);
        corr_est.setInputTarget(features2);

        pcl::Correspondences correspondences;
        corr_est.determineReciprocalCorrespondences(correspondences);

        // Calculate similarity based on correspondence ratio and quality
        if (correspondences.empty())
        {
            return 0.0;
        }

        double total_distance = 0.0;
        for (const auto& corr : correspondences)
        {
            total_distance += corr.distance;
        }

        double avg_distance = total_distance / correspondences.size();
        double correspondence_ratio = static_cast<double>(correspondences.size()) / std::min(features1->size(), features2->size());
        
        // Combine correspondence ratio and feature distance quality
        double similarity = correspondence_ratio * (1.0 / (1.0 + avg_distance));
        
        return std::min(1.0, similarity);
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error calculating feature similarity: %s", e.what());
        return 0.0;
    }
}

bool MapBuilder::verifyGeometricConsistency(const KeyFrame& current, const KeyFrame& candidate)
{
    try
    {
        if (current.keypoints->empty() || candidate.keypoints->empty())
        {
            return false;
        }

        // Use RANSAC for geometric verification
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac;
        pcl::Correspondences correspondences;
        
        // Establish correspondences between keypoints
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
        corr_est.setInputSource(current.keypoints);
        corr_est.setInputTarget(candidate.keypoints);
        corr_est.determineCorrespondences(correspondences);

        if (correspondences.size() < 10)
        {
            return false;
        }

        ransac.setInputSource(current.keypoints);
        ransac.setInputTarget(candidate.keypoints);
        ransac.setInlierThreshold(0.1);
        ransac.setMaximumIterations(1000);
        
        pcl::Correspondences inlier_correspondences;
        ransac.getRemainingCorrespondences(correspondences, inlier_correspondences);

        double inlier_ratio = static_cast<double>(inlier_correspondences.size()) / correspondences.size();
        
        RCLCPP_DEBUG(this->get_logger(), "Geometric verification: %zu/%zu inliers (%.2f%%)", 
                     inlier_correspondences.size(), correspondences.size(), inlier_ratio * 100);

        return inlier_ratio > 0.3; // At least 30% inliers
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error in geometric verification: %s", e.what());
        return false;
    }
}

void MapBuilder::applyLoopClosureCorrection(const KeyFrame& current, const KeyFrame& candidate)
{
    try
    {
        // Simple correction: adjust map probabilities in the loop closure region
        // In a full SLAM implementation, this would involve pose graph optimization
        
        double correction_radius = 3.0; // meters
        geometry_msgs::msg::Point center;
        center.x = (current.pose.position.x + candidate.pose.position.x) / 2.0;
        center.y = (current.pose.position.y + candidate.pose.position.y) / 2.0;
        center.z = (current.pose.position.z + candidate.pose.position.z) / 2.0;

        std::lock_guard<std::mutex> lock(map_mutex_);
        
        // Increase confidence in the loop closure region
        int center_gx = static_cast<int>((center.x - map_origin_x_) / map_resolution_);
        int center_gy = static_cast<int>((center.y - map_origin_y_) / map_resolution_);
        int radius_cells = static_cast<int>(correction_radius / map_resolution_);

        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            {
                int gx = center_gx + dx;
                int gy = center_gy + dy;

                if (isValidGridCoordinate(gx, gy))
                {
                    int index = gridIndex(gx, gy);
                    if (probability_map_[index] > 0.5)
                    {
                        probability_map_[index] = std::min(occupancy_max_probability_, probability_map_[index] + 0.1);
                        occupancy_map_[index] = (probability_map_[index] > 0.65) ? 100 : 0;
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Applied loop closure correction at (%.2f, %.2f)", center.x, center.y);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error applying loop closure correction: %s", e.what());
    }
}

void MapBuilder::updateEnhancedOccupancyGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
    try
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        for (const auto& point : points->points)
        {
            // Convert world coordinates to grid coordinates
            int grid_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
            int grid_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_);

            if (!isValidGridCoordinate(grid_x, grid_y))
            {
                continue;
            }

            int index = gridIndex(grid_x, grid_y);

            // Update height map (take maximum height)
            if (std::isnan(height_map_[index]) || point.z > height_map_[index])
            {
                height_map_[index] = point.z;
            }

            // Enhanced probabilistic update
            if (point.z >= min_obstacle_height_ && point.z <= max_obstacle_height_)
            {
                // Hit: increase probability
                probability_map_[index] = updateProbability(probability_map_[index], occupancy_hit_probability_);
                occupancy_map_[index] = (probability_map_[index] > 0.65) ? 100 : 0;
            }
            else
            {
                // Miss: decrease probability (but less aggressively)
                probability_map_[index] = updateProbability(probability_map_[index], occupancy_miss_probability_);
                occupancy_map_[index] = (probability_map_[index] < 0.35) ? 0 : occupancy_map_[index];
            }
        }

        // Enhanced ray tracing with probabilistic updates
        if (current_robot_pose_)
        {
            enhancedRayTraceFreespace(points);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error updating enhanced occupancy grid: %s", e.what());
    }
}

double MapBuilder::updateProbability(double current_prob, double sensor_prob)
{
    // Bayesian update: P(occ|z) = [P(z|occ) * P(occ)] / [P(z|occ) * P(occ) + P(z|free) * P(free)]
    double odds_ratio = sensor_prob / (1.0 - sensor_prob);
    double current_odds = current_prob / (1.0 - current_prob);
    double new_odds = odds_ratio * current_odds;
    double new_prob = new_odds / (1.0 + new_odds);
    
    // Clamp to avoid extreme values
    return std::max(occupancy_min_probability_, std::min(occupancy_max_probability_, new_prob));
}

void MapBuilder::enhancedRayTraceFreespace(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
    try
    {
        if (!current_robot_pose_)
        {
            return;
        }

        // Convert robot position to grid coordinates
        int robot_gx = static_cast<int>((current_robot_pose_->pose.position.x - map_origin_x_) / map_resolution_);
        int robot_gy = static_cast<int>((current_robot_pose_->pose.position.y - map_origin_y_) / map_resolution_);

        if (!isValidGridCoordinate(robot_gx, robot_gy))
        {
            return;
        }

        // Enhanced ray tracing with adaptive sampling
        size_t step = std::max(static_cast<size_t>(1), points->size() / 200);
        for (size_t i = 0; i < points->size(); i += step)
        {
            const auto& point = points->points[i];
            
            int point_gx = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
            int point_gy = static_cast<int>((point.y - map_origin_y_) / map_resolution_);

            if (!isValidGridCoordinate(point_gx, point_gy))
            {
                continue;
            }

            // Bresenham's line algorithm for ray tracing
            auto cells = bresenhamLine(robot_gx, robot_gy, point_gx, point_gy);

            // Mark all cells except the endpoint as free (probabilistically)
            for (size_t j = 0; j < cells.size() - 1; ++j)
            {
                int cell_x = cells[j].first;
                int cell_y = cells[j].second;

                if (isValidGridCoordinate(cell_x, cell_y))
                {
                    int index = gridIndex(cell_x, cell_y);
                    
                    // Only update if not already marked as occupied with high confidence
                    if (probability_map_[index] < 0.8)
                    {
                        probability_map_[index] = updateProbability(probability_map_[index], occupancy_miss_probability_);
                        occupancy_map_[index] = (probability_map_[index] < 0.35) ? 0 : occupancy_map_[index];
                    }
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in enhanced ray tracing: %s", e.what());
    }
}

void MapBuilder::manageMemory()
{
    try
    {
        static auto last_cleanup = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_cleanup).count();

        if (elapsed < memory_cleanup_interval_)
        {
            return;
        }

        // Count non-unknown cells
        int occupied_cells = 0;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            for (int value : occupancy_map_)
            {
                if (value != -1)
                {
                    occupied_cells++;
                }
            }
        }

        if (occupied_cells > max_map_points_)
        {
            RCLCPP_WARN(this->get_logger(), "Map memory limit reached (%d/%d cells), performing cleanup", 
                       occupied_cells, max_map_points_);
            
            // Clean up old/uncertain areas
            performMemoryCleanup();
        }

        // Clean up old keyframes
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            if (static_cast<int>(keyframes_.size()) > keyframe_buffer_size_)
            {
                size_t to_remove = keyframes_.size() - keyframe_buffer_size_;
                keyframes_.erase(keyframes_.begin(), keyframes_.begin() + to_remove);
            }
        }

        last_cleanup = now;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in memory management: %s", e.what());
    }
}

void MapBuilder::performMemoryCleanup()
{
    try
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        
        // Reset cells with low confidence back to unknown
        for (size_t i = 0; i < probability_map_.size(); ++i)
        {
            if (probability_map_[i] > 0.4 && probability_map_[i] < 0.6)  // Uncertain cells
            {
                probability_map_[i] = 0.5;
                occupancy_map_[i] = -1;
                height_map_[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        RCLCPP_INFO(this->get_logger(), "Memory cleanup completed");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in memory cleanup: %s", e.what());
    }
}

} // namespace map_builder