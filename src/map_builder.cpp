#include "map_builder/map_builder.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

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
    this->declare_parameter<double>("map_resolution", 0.05);
    this->declare_parameter<int>("map_width", 400);
    this->declare_parameter<int>("map_height", 400);
    this->declare_parameter<double>("map_origin_x", -10.0);
    this->declare_parameter<double>("map_origin_y", -10.0);
    this->declare_parameter<double>("robot_radius", 0.3);
    this->declare_parameter<double>("min_obstacle_height", 0.1);
    this->declare_parameter<double>("max_obstacle_height", 2.0);
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
}

void MapBuilder::initializeMap()
{
    // Initialize occupancy map (-1 = unknown, 0 = free, 100 = occupied)
    occupancy_map_.assign(map_width_ * map_height_, -1);
    
    // Initialize height map (NaN = unknown)
    height_map_.assign(map_width_ * map_height_, std::numeric_limits<float>::quiet_NaN());

    RCLCPP_INFO(this->get_logger(), "Map initialized with %d cells", 
                static_cast<int>(occupancy_map_.size()));
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

        // Update occupancy grid
        updateOccupancyGrid(transformed_points);

        RCLCPP_DEBUG(this->get_logger(), "Updated map with %zu points", transformed_points->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud for mapping: %s", e.what());
    }
}

void MapBuilder::robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_robot_pose_ = msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapBuilder::transformPointsToMapFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr points, const std_msgs::msg::Header& header)
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

} // namespace map_builder