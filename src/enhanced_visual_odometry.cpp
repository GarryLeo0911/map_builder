#include "map_builder/enhanced_visual_odometry.hpp"
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace map_builder
{

EnhancedVisualOdometry::EnhancedVisualOdometry()
    : Node("enhanced_visual_odometry"),
      cumulative_transform_(Eigen::Matrix4d::Identity()),
      odometry_initialized_(false),
      current_frame_id_(0),
      imu_initialized_(false),
      has_rgb_data_(false),
      has_depth_data_(false),
      has_pointcloud_data_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Enhanced Visual Odometry for OAK-D");

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Declare and get parameters
    declareParameters();
    getParameters();

    // Initialize feature detector (ORB for real-time performance)
    feature_detector_ = cv::ORB::create(max_features_, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, feature_detector_threshold_);
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    // Initialize subscribers
    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "oak/rgb/image_raw", 10,
        std::bind(&EnhancedVisualOdometry::rgbImageCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "oak/stereo/depth", 10,
        std::bind(&EnhancedVisualOdometry::depthImageCallback, this, std::placeholders::_1));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "oak/points", 10,
        std::bind(&EnhancedVisualOdometry::pointCloudCallback, this, std::placeholders::_1));

    if (enable_imu_fusion_)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "oak/imu", 10,
            std::bind(&EnhancedVisualOdometry::imuCallback, this, std::placeholders::_1));
    }

    // Initialize publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "enhanced_visual_odometry/odometry", 10);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "enhanced_visual_odometry/pose", 10);

    // Initialize timer for odometry processing
    odometry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),  // ~30 Hz
        std::bind(&EnhancedVisualOdometry::processVisualOdometry, this));

    // Initialize state
    velocity_.setZero();
    bias_gyro_.setZero();
    bias_accel_.setZero();

    RCLCPP_INFO(this->get_logger(), "Enhanced Visual Odometry initialized with %s IMU fusion", 
                enable_imu_fusion_ ? "enabled" : "disabled");
}

void EnhancedVisualOdometry::declareParameters()
{
    // Feature detection parameters
    this->declare_parameter<double>("feature_detector_threshold", 20.0);
    this->declare_parameter<int>("max_features", 1000);
    this->declare_parameter<double>("match_ratio_threshold", 0.75);
    this->declare_parameter<int>("min_matches", 50);
    this->declare_parameter<double>("ransac_threshold", 3.0);
    this->declare_parameter<double>("max_translation_per_frame", 1.0);
    this->declare_parameter<double>("max_rotation_per_frame", 0.5);

    // IMU fusion parameters
    this->declare_parameter<bool>("enable_imu_fusion", true);
    this->declare_parameter<double>("imu_weight", 0.3);
    this->declare_parameter<double>("gravity_magnitude", 9.81);

    // Point cloud odometry parameters
    this->declare_parameter<double>("voxel_size", 0.02);
    this->declare_parameter<double>("icp_max_correspondence_distance", 0.05);
    this->declare_parameter<int>("icp_max_iterations", 50);
    this->declare_parameter<double>("icp_transformation_epsilon", 1e-6);

    // Frame management
    this->declare_parameter<int>("max_frame_buffer_size", 10);
}

void EnhancedVisualOdometry::getParameters()
{
    this->get_parameter("feature_detector_threshold", feature_detector_threshold_);
    this->get_parameter("max_features", max_features_);
    this->get_parameter("match_ratio_threshold", match_ratio_threshold_);
    this->get_parameter("min_matches", min_matches_);
    this->get_parameter("ransac_threshold", ransac_threshold_);
    this->get_parameter("max_translation_per_frame", max_translation_per_frame_);
    this->get_parameter("max_rotation_per_frame", max_rotation_per_frame_);

    this->get_parameter("enable_imu_fusion", enable_imu_fusion_);
    this->get_parameter("imu_weight", imu_weight_);
    this->get_parameter("gravity_magnitude", gravity_magnitude_);

    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("icp_max_correspondence_distance", icp_max_correspondence_distance_);
    this->get_parameter("icp_max_iterations", icp_max_iterations_);
    this->get_parameter("icp_transformation_epsilon", icp_transformation_epsilon_);

    this->get_parameter("max_frame_buffer_size", max_frame_buffer_size_);

    RCLCPP_INFO(this->get_logger(), "Enhanced VO parameters - max_features: %d, IMU fusion: %s, voxel_size: %.3f", 
                max_features_, enable_imu_fusion_ ? "enabled" : "disabled", voxel_size_);
}

void EnhancedVisualOdometry::rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat rgb_image;
        if (convertRosImageToMat(msg, rgb_image, sensor_msgs::image_encodings::BGR8))
        {
            std::lock_guard<std::mutex> lock(odometry_mutex_);
            latest_rgb_image_ = rgb_image.clone();
            latest_data_timestamp_ = msg->header.stamp;
            has_rgb_data_ = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert RGB image");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "RGB image conversion failed: %s", e.what());
    }
}

void EnhancedVisualOdometry::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat depth_image;
        if (convertRosImageToMat(msg, depth_image, sensor_msgs::image_encodings::TYPE_16UC1))
        {
            std::lock_guard<std::mutex> lock(odometry_mutex_);
            latest_depth_image_ = depth_image.clone();
            has_depth_data_ = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert depth image");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Depth image conversion failed: %s", e.what());
    }
}

void EnhancedVisualOdometry::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        std::lock_guard<std::mutex> lock(odometry_mutex_);
        latest_point_cloud_ = preprocessPointCloud(cloud);
        has_pointcloud_data_ = true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Point cloud conversion failed: %s", e.what());
    }
}

void EnhancedVisualOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!enable_imu_fusion_) return;

    try
    {
        std::lock_guard<std::mutex> lock(odometry_mutex_);
        
        if (imu_initialized_)
        {
            double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_data_.header.stamp)).seconds();
            if (dt > 0.0 && dt < 0.1)  // Reasonable time delta
            {
                integrateIMU(*msg, dt);
            }
        }
        else
        {
            imu_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "IMU initialized");
        }
        
        last_imu_data_ = *msg;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "IMU processing failed: %s", e.what());
    }
}

void EnhancedVisualOdometry::processVisualOdometry()
{
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    
    // Check if we have all required data
    if (!has_rgb_data_ || !has_depth_data_ || !has_pointcloud_data_)
    {
        return;
    }

    try
    {
        // Process current frame
        VisualFrame current_frame = processFrame(latest_rgb_image_, latest_depth_image_, 
                                                latest_point_cloud_, latest_data_timestamp_);

        if (frame_buffer_.empty())
        {
            // First frame - initialize
            frame_buffer_.push_back(current_frame);
            odometry_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Enhanced visual odometry initialized with frame %d", current_frame.frame_id);
            return;
        }

        // Get previous frame
        const VisualFrame& previous_frame = frame_buffer_.back();

        // Estimate motion using multiple approaches
        Eigen::Matrix4d visual_motion = Eigen::Matrix4d::Identity();
        bool visual_success = false;

        // Try feature-based visual odometry first
        if (current_frame.keypoints.size() >= min_matches_ && previous_frame.keypoints.size() >= min_matches_)
        {
            visual_motion = estimateMotionFromFeatures(previous_frame, current_frame);
            visual_success = validateTransformation(visual_motion);
            
            if (visual_success)
            {
                RCLCPP_DEBUG(this->get_logger(), "Feature-based motion estimation successful");
            }
        }

        // Fallback to point cloud odometry if feature-based fails
        if (!visual_success)
        {
            visual_motion = estimateMotionFromPointClouds(previous_frame, current_frame);
            visual_success = validateTransformation(visual_motion);
            
            if (visual_success)
            {
                RCLCPP_DEBUG(this->get_logger(), "Point cloud-based motion estimation used as fallback");
            }
        }

        if (visual_success)
        {
            // IMU fusion if available
            if (enable_imu_fusion_ && imu_initialized_)
            {
                double dt = (current_frame.timestamp - previous_frame.timestamp).seconds();
                if (dt > 0.0 && dt < 0.2)  // Reasonable time delta
                {
                    Eigen::Matrix4d imu_motion = predictMotionFromIMU(dt);
                    fuseVisualIMU(visual_motion, imu_motion, dt);
                }
                else
                {
                    updateOdometry(visual_motion, current_frame.timestamp);
                }
            }
            else
            {
                updateOdometry(visual_motion, current_frame.timestamp);
            }

            // Add current frame to buffer
            frame_buffer_.push_back(current_frame);
            
            // Maintain buffer size
            if (static_cast<int>(frame_buffer_.size()) > max_frame_buffer_size_)
            {
                frame_buffer_.pop_front();
            }

            // Publish odometry
            publishOdometry();
            broadcastTransform(current_frame.timestamp);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Motion estimation failed, skipping frame %d", current_frame.frame_id);
        }

        // Reset data flags
        has_rgb_data_ = false;
        has_depth_data_ = false;
        has_pointcloud_data_ = false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in visual odometry processing: %s", e.what());
    }
}

VisualFrame EnhancedVisualOdometry::processFrame(const cv::Mat& rgb_image, const cv::Mat& depth_image, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                                const rclcpp::Time& timestamp)
{
    VisualFrame frame;
    frame.timestamp = timestamp;
    frame.frame_id = current_frame_id_++;
    frame.rgb_image = rgb_image.clone();
    frame.depth_image = depth_image.clone();
    frame.point_cloud = point_cloud;
    frame.has_imu = imu_initialized_;
    
    if (frame.has_imu)
    {
        frame.imu_data = last_imu_data_;
    }

    // Extract features
    cv::Mat gray_image;
    cv::cvtColor(rgb_image, gray_image, cv::COLOR_BGR2GRAY);
    
    frame.keypoints = detectFeatures(gray_image);
    frame.descriptors = extractDescriptors(gray_image, frame.keypoints);

    return frame;
}

std::vector<cv::KeyPoint> EnhancedVisualOdometry::detectFeatures(const cv::Mat& image)
{
    std::vector<cv::KeyPoint> keypoints;
    
    try
    {
        feature_detector_->detect(image, keypoints);
        
        // Filter keypoints to ensure good distribution
        if (keypoints.size() > static_cast<size_t>(max_features_))
        {
            // Sort by response and keep the best ones
            std::sort(keypoints.begin(), keypoints.end(),
                     [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                         return a.response > b.response;
                     });
            keypoints.resize(max_features_);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Feature detection failed: %s", e.what());
    }

    return keypoints;
}

cv::Mat EnhancedVisualOdometry::extractDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
    cv::Mat descriptors;
    
    try
    {
        if (!keypoints.empty())
        {
            feature_detector_->compute(image, keypoints, descriptors);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Descriptor extraction failed: %s", e.what());
    }

    return descriptors;
}

std::vector<cv::DMatch> EnhancedVisualOdometry::matchFeatures(const cv::Mat& desc1, const cv::Mat& desc2)
{
    std::vector<cv::DMatch> good_matches;

    try
    {
        if (desc1.empty() || desc2.empty())
        {
            return good_matches;
        }

        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher_->knnMatch(desc1, desc2, knn_matches, 2);

        // Apply Lowe's ratio test
        for (const auto& match_pair : knn_matches)
        {
            if (match_pair.size() >= 2)
            {
                if (match_pair[0].distance < match_ratio_threshold_ * match_pair[1].distance)
                {
                    good_matches.push_back(match_pair[0]);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Feature matching failed: %s", e.what());
    }

    return good_matches;
}

Eigen::Matrix4d EnhancedVisualOdometry::estimateMotionFromFeatures(const VisualFrame& frame1, const VisualFrame& frame2)
{
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    try
    {
        if (frame1.descriptors.empty() || frame2.descriptors.empty())
        {
            return transformation;
        }

        // Match features between frames
        std::vector<cv::DMatch> matches = matchFeatures(frame1.descriptors, frame2.descriptors);

        if (static_cast<int>(matches.size()) < min_matches_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Insufficient feature matches: %zu < %d", matches.size(), min_matches_);
            return transformation;
        }

        // Extract matched 3D points (using depth information)
        std::vector<cv::Point3f> points1, points2;
        
        for (const auto& match : matches)
        {
            const cv::KeyPoint& kp1 = frame1.keypoints[match.queryIdx];
            const cv::KeyPoint& kp2 = frame2.keypoints[match.trainIdx];

            // Get depth values
            uint16_t depth1 = frame1.depth_image.at<uint16_t>(static_cast<int>(kp1.pt.y), static_cast<int>(kp1.pt.x));
            uint16_t depth2 = frame2.depth_image.at<uint16_t>(static_cast<int>(kp2.pt.y), static_cast<int>(kp2.pt.x));

            if (depth1 > 0 && depth2 > 0)
            {
                // Convert to 3D points (assuming camera intrinsics)
                // Note: These should be actual camera intrinsics from OAK-D calibration
                float fx = 640.0f;  // These should come from camera_info topic
                float fy = 640.0f;
                float cx = 320.0f;
                float cy = 240.0f;
                float depth_scale = 1000.0f;  // OAK-D depth scale

                float z1 = depth1 / depth_scale;
                float x1 = (kp1.pt.x - cx) * z1 / fx;
                float y1 = (kp1.pt.y - cy) * z1 / fy;

                float z2 = depth2 / depth_scale;
                float x2 = (kp2.pt.x - cx) * z2 / fx;
                float y2 = (kp2.pt.y - cy) * z2 / fy;

                points1.emplace_back(x1, y1, z1);
                points2.emplace_back(x2, y2, z2);
            }
        }

        if (points1.size() < static_cast<size_t>(min_matches_))
        {
            RCLCPP_DEBUG(this->get_logger(), "Insufficient 3D point correspondences: %zu", points1.size());
            return transformation;
        }

        // Estimate transformation using RANSAC
        cv::Mat rvec, tvec, inliers;
        bool success = cv::solvePnPRansac(points1, points2, cv::Mat::eye(3, 3, CV_32F), cv::Mat::zeros(4, 1, CV_32F),
                                         rvec, tvec, false, 100, ransac_threshold_, 0.99, inliers);

        if (success && inliers.rows >= min_matches_)
        {
            // Convert to transformation matrix
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);

            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    transformation(i, j) = rotation_matrix.at<double>(i, j);
                }
                transformation(i, 3) = tvec.at<double>(i);
            }

            RCLCPP_DEBUG(this->get_logger(), "Feature-based motion estimation: %d inliers from %zu matches", 
                        inliers.rows, matches.size());
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Feature-based motion estimation failed: %s", e.what());
    }

    return transformation;
}

Eigen::Matrix4d EnhancedVisualOdometry::estimateMotionFromPointClouds(const VisualFrame& frame1, const VisualFrame& frame2)
{
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    try
    {
        if (!frame1.point_cloud || !frame2.point_cloud || 
            frame1.point_cloud->empty() || frame2.point_cloud->empty())
        {
            return transformation;
        }

        // Use Generalized ICP for better performance with structured point clouds
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(frame2.point_cloud);
        gicp.setInputTarget(frame1.point_cloud);
        gicp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
        gicp.setMaximumIterations(icp_max_iterations_);
        gicp.setTransformationEpsilon(icp_transformation_epsilon_);

        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        gicp.align(aligned_cloud);

        if (gicp.hasConverged())
        {
            Eigen::Matrix4f transform_f = gicp.getFinalTransformation();
            transformation = transform_f.cast<double>();
            
            RCLCPP_DEBUG(this->get_logger(), "Point cloud motion estimation: fitness score = %f", 
                        gicp.getFitnessScore());
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Point cloud motion estimation failed: %s", e.what());
    }

    return transformation;
}

void EnhancedVisualOdometry::integrateIMU(const sensor_msgs::msg::Imu& imu_data, double dt)
{
    try
    {
        // Simple IMU integration (in practice, this should be more sophisticated)
        Eigen::Vector3d accel(imu_data.linear_acceleration.x, 
                             imu_data.linear_acceleration.y, 
                             imu_data.linear_acceleration.z);
        
        Eigen::Vector3d gyro(imu_data.angular_velocity.x,
                            imu_data.angular_velocity.y,
                            imu_data.angular_velocity.z);

        // Remove bias (simplified)
        accel -= bias_accel_;
        gyro -= bias_gyro_;

        // Remove gravity (simplified - assumes level start)
        accel.z() -= gravity_magnitude_;

        // Integrate velocity
        velocity_ += accel * dt;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "IMU integration failed: %s", e.what());
    }
}

Eigen::Matrix4d EnhancedVisualOdometry::predictMotionFromIMU(double dt)
{
    Eigen::Matrix4d prediction = Eigen::Matrix4d::Identity();

    try
    {
        if (!imu_initialized_ || dt <= 0.0)
        {
            return prediction;
        }

        // Simple prediction based on integrated velocity
        prediction(0, 3) = velocity_.x() * dt;
        prediction(1, 3) = velocity_.y() * dt;
        prediction(2, 3) = velocity_.z() * dt;

        // For rotation, we would need more sophisticated integration
        // This is a simplified version
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "IMU motion prediction failed: %s", e.what());
    }

    return prediction;
}

void EnhancedVisualOdometry::fuseVisualIMU(const Eigen::Matrix4d& visual_motion, const Eigen::Matrix4d& imu_motion, double dt)
{
    try
    {
        // Simple weighted fusion (in practice, use Kalman filter or similar)
        Eigen::Matrix4d fused_motion = (1.0 - imu_weight_) * visual_motion + imu_weight_ * imu_motion;
        
        updateOdometry(fused_motion, latest_data_timestamp_);
        
        RCLCPP_DEBUG(this->get_logger(), "Fused visual-IMU motion with weight %.2f", imu_weight_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Visual-IMU fusion failed: %s", e.what());
    }
}

bool EnhancedVisualOdometry::validateTransformation(const Eigen::Matrix4d& transform)
{
    try
    {
        // Check for NaN or infinite values
        if (!transform.allFinite())
        {
            return false;
        }

        // Extract translation and rotation
        Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
        Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);

        // Check translation magnitude
        if (translation.norm() > max_translation_per_frame_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Large translation rejected: %.3f > %.3f", 
                        translation.norm(), max_translation_per_frame_);
            return false;
        }

        // Check rotation magnitude
        Eigen::AngleAxisd angle_axis(rotation);
        if (std::abs(angle_axis.angle()) > max_rotation_per_frame_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Large rotation rejected: %.3f > %.3f", 
                        std::abs(angle_axis.angle()), max_rotation_per_frame_);
            return false;
        }

        // Check if rotation matrix is valid
        if (std::abs(rotation.determinant() - 1.0) > 0.1)
        {
            return false;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Transformation validation failed: %s", e.what());
        return false;
    }
}

void EnhancedVisualOdometry::updateOdometry(const Eigen::Matrix4d& transform, const rclcpp::Time& timestamp)
{
    try
    {
        // Apply transformation to cumulative transform
        cumulative_transform_ = cumulative_transform_ * transform;

        // Update pose
        current_pose_ = poseFromMatrix4d(cumulative_transform_);

        // Update odometry message
        current_odometry_.header.stamp = timestamp;
        current_odometry_.header.frame_id = "map";
        current_odometry_.child_frame_id = "base_link";
        current_odometry_.pose.pose = current_pose_;
        
        // Simple velocity estimation (could be improved)
        static rclcpp::Time last_update_time = timestamp;
        double dt = (timestamp - last_update_time).seconds();
        if (dt > 0.0)
        {
            Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
            current_odometry_.twist.twist.linear.x = translation.x() / dt;
            current_odometry_.twist.twist.linear.y = translation.y() / dt;
            current_odometry_.twist.twist.linear.z = translation.z() / dt;
        }
        last_update_time = timestamp;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry update failed: %s", e.what());
    }
}

void EnhancedVisualOdometry::publishOdometry()
{
    try
    {
        // Publish odometry
        odom_pub_->publish(current_odometry_);

        // Publish pose
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = current_odometry_.header;
        pose_stamped.pose = current_pose_;
        pose_pub_->publish(pose_stamped);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry publishing failed: %s", e.what());
    }
}

void EnhancedVisualOdometry::broadcastTransform(const rclcpp::Time& timestamp)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        transform_stamped.header.stamp = timestamp;
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        
        transform_stamped.transform.translation.x = current_pose_.position.x;
        transform_stamped.transform.translation.y = current_pose_.position.y;
        transform_stamped.transform.translation.z = current_pose_.position.z;
        transform_stamped.transform.rotation = current_pose_.orientation;
        
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform broadcast failed: %s", e.what());
    }
}

geometry_msgs::msg::Pose EnhancedVisualOdometry::poseFromMatrix4d(const Eigen::Matrix4d& matrix)
{
    geometry_msgs::msg::Pose pose;
    
    // Translation
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    
    // Rotation (rotation matrix to quaternion)
    Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rotation);
    
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}

Eigen::Matrix4d EnhancedVisualOdometry::matrix4dFromPose(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    
    // Translation
    matrix(0, 3) = pose.position.x;
    matrix(1, 3) = pose.position.y;
    matrix(2, 3) = pose.position.z;
    
    // Rotation (quaternion to rotation matrix)
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, 
                        pose.orientation.y, pose.orientation.z);
    matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    return matrix;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EnhancedVisualOdometry::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        if (cloud->empty())
        {
            return filtered_cloud;
        }

        // Remove NaN points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);

        // Voxel downsampling
        if (!filtered_cloud->empty() && voxel_size_ > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(filtered_cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*filtered_cloud);
        }

        // Statistical outlier removal
        if (filtered_cloud->size() > 10)
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(filtered_cloud);
            sor.setMeanK(20);
            sor.setStddevMulThresh(2.0);
            sor.filter(*filtered_cloud);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Point cloud preprocessing failed: %s", e.what());
        return cloud;
    }

    return filtered_cloud;
}

// Manual image conversion functions (temporary replacement for cv_bridge)
bool EnhancedVisualOdometry::convertRosImageToMat(const sensor_msgs::msg::Image::SharedPtr& msg, cv::Mat& output, const std::string& encoding)
{
    try
    {
        // Handle BGR8 encoding
        if (encoding == sensor_msgs::image_encodings::BGR8)
        {
            if (msg->encoding == "bgr8" || msg->encoding == "rgb8")
            {
                output = cv::Mat(msg->height, msg->width, CV_8UC3, (void*)msg->data.data()).clone();
                if (msg->encoding == "rgb8")
                {
                    cv::cvtColor(output, output, cv::COLOR_RGB2BGR);
                }
                return true;
            }
            else if (msg->encoding == "mono8")
            {
                cv::Mat mono = cv::Mat(msg->height, msg->width, CV_8UC1, (void*)msg->data.data());
                cv::cvtColor(mono, output, cv::COLOR_GRAY2BGR);
                return true;
            }
        }
        // Handle 16UC1 encoding (depth images)
        else if (encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        {
            if (msg->encoding == "16UC1" || msg->encoding == "mono16")
            {
                output = cv::Mat(msg->height, msg->width, CV_16UC1, (void*)msg->data.data()).clone();
                return true;
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s -> %s", msg->encoding.c_str(), encoding.c_str());
        return false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Image conversion error: %s", e.what());
        return false;
    }
}

cv::Mat EnhancedVisualOdometry::rosImageToMat(const sensor_msgs::msg::Image::SharedPtr& msg)
{
    cv::Mat result;
    if (convertRosImageToMat(msg, result, msg->encoding))
    {
        return result;
    }
    return cv::Mat();
}

} // namespace map_builder