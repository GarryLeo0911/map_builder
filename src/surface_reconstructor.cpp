#include "map_builder/surface_reconstructor.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>

namespace map_builder
{

SurfaceReconstructor::SurfaceReconstructor()
    : Node("surface_reconstructor")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Surface Reconstructor");

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare and get parameters
    declareParameters();
    getParameters();

    // Initialize subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "map_builder/accumulated_points", 10,
        std::bind(&SurfaceReconstructor::pointcloudCallback, this, std::placeholders::_1));

    // Initialize publishers
    mesh_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "map_builder/mesh_markers", 10);
    
    surface_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "map_builder/surface_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Surface Reconstructor initialized");
}

void SurfaceReconstructor::declareParameters()
{
    this->declare_parameter<double>("mesh_resolution", 0.1);
    this->declare_parameter<double>("clustering_tolerance", 0.2);
    this->declare_parameter<int>("clustering_min_cluster_size", 50);
    this->declare_parameter<int>("clustering_max_cluster_size", 25000);
    this->declare_parameter<double>("convex_hull_alpha", 0.05);
}

void SurfaceReconstructor::getParameters()
{
    this->get_parameter("mesh_resolution", mesh_resolution_);
    this->get_parameter("clustering_tolerance", clustering_tolerance_);
    this->get_parameter("clustering_min_cluster_size", clustering_min_cluster_size_);
    this->get_parameter("clustering_max_cluster_size", clustering_max_cluster_size_);
    this->get_parameter("convex_hull_alpha", convex_hull_alpha_);

    RCLCPP_INFO(this->get_logger(), "Parameters loaded - mesh resolution: %.3f, clustering tolerance: %.3f", 
                mesh_resolution_, clustering_tolerance_);
}

void SurfaceReconstructor::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        // Convert ROS PointCloud2 to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty() || static_cast<int>(cloud->size()) < clustering_min_cluster_size_)
        {
            RCLCPP_WARN(this->get_logger(), "Point cloud too small for surface reconstruction");
            return;
        }

        // Cluster points to identify surfaces
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = clusterPoints(cloud);

        if (clusters.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No clusters found for surface reconstruction");
            return;
        }

        // Generate mesh markers for each cluster
        auto mesh_markers = generateMeshMarkers(clusters, msg->header);

        // Generate surface markers
        auto surface_markers = generateSurfaceMarkers(clusters, msg->header);

        // Publish markers
        if (!mesh_markers.markers.empty())
        {
            mesh_pub_->publish(mesh_markers);
        }

        if (!surface_markers.markers.empty())
        {
            surface_pub_->publish(surface_markers);
        }

        RCLCPP_DEBUG(this->get_logger(), "Generated surface reconstruction with %zu clusters", clusters.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in surface reconstruction: %s", e.what());
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> SurfaceReconstructor::clusterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    try
    {
        // Create KdTree for clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // Euclidean cluster extraction
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(clustering_tolerance_);
        ec.setMinClusterSize(clustering_min_cluster_size_);
        ec.setMaxClusterSize(clustering_max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Extract clusters
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));
            
            extract.setInputCloud(cloud);
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*cluster);

            if (!cluster->empty())
            {
                clusters.push_back(cluster);
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters from %zu points", 
                     clusters.size(), cloud->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error clustering points: %s", e.what());
    }

    return clusters;
}

visualization_msgs::msg::MarkerArray SurfaceReconstructor::generateMeshMarkers(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, 
    const std_msgs::msg::Header& header)
{
    visualization_msgs::msg::MarkerArray marker_array;

    try
    {
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const auto& cluster = clusters[i];

            if (cluster->size() < 4) // Need at least 4 points for mesh
            {
                continue;
            }

            // Create mesh marker
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "mesh_surfaces";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set marker properties
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.r = 0.0;
            marker.color.g = 0.8;
            marker.color.b = 0.0;
            marker.color.a = 0.6;

            // Generate convex hull for the cluster
            pcl::PointCloud<pcl::PointXYZ>::Ptr hull = generateConvexHull(cluster);

            if (hull && hull->size() >= 3)
            {
                // Simple fan triangulation from first vertex
                for (size_t j = 1; j < hull->size() - 1; ++j)
                {
                    marker.points.push_back(pclToGeometryPoint(hull->points[0]));
                    marker.points.push_back(pclToGeometryPoint(hull->points[j]));
                    marker.points.push_back(pclToGeometryPoint(hull->points[j + 1]));
                }
            }

            if (!marker.points.empty())
            {
                marker_array.markers.push_back(marker);
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error generating mesh markers: %s", e.what());
    }

    return marker_array;
}

visualization_msgs::msg::MarkerArray SurfaceReconstructor::generateSurfaceMarkers(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, 
    const std_msgs::msg::Header& header)
{
    visualization_msgs::msg::MarkerArray marker_array;

    try
    {
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const auto& cluster = clusters[i];

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "surface_points";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set marker properties
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            // Color based on cluster height
            double sum_height = 0.0;
            for (const auto& point : cluster->points)
            {
                sum_height += point.z;
            }
            double avg_height = sum_height / cluster->size();
            
            // Normalize height between -2 and 2 meters to color range 0-1
            double normalized_height = std::max(0.0, std::min(1.0, (avg_height + 2.0) / 4.0));

            marker.color.r = normalized_height;
            marker.color.g = 0.5;
            marker.color.b = 1.0 - normalized_height;
            marker.color.a = 0.8;

            // Add points (subsample for performance)
            size_t step = std::max(static_cast<size_t>(1), cluster->size() / 100);
            for (size_t j = 0; j < cluster->size(); j += step)
            {
                marker.points.push_back(pclToGeometryPoint(cluster->points[j]));
            }

            if (!marker.points.empty())
            {
                marker_array.markers.push_back(marker);
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error generating surface markers: %s", e.what());
    }

    return marker_array;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SurfaceReconstructor::generateConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
        if (cluster->size() < 3)
        {
            return hull;
        }

        // Use a simpler approach: find boundary points in 2D (approximate convex hull)
        std::vector<pcl::PointXYZ> boundary_points;
        
        // Find extreme points in X and Y directions
        auto min_x_it = std::min_element(cluster->points.begin(), cluster->points.end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.x < b.x; });
        auto max_x_it = std::max_element(cluster->points.begin(), cluster->points.end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.x < b.x; });
        auto min_y_it = std::min_element(cluster->points.begin(), cluster->points.end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.y < b.y; });
        auto max_y_it = std::max_element(cluster->points.begin(), cluster->points.end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.y < b.y; });

        // Add unique extreme points
        std::set<std::tuple<float, float, float>> unique_points;
        
        auto add_unique_point = [&](const pcl::PointXYZ& point) {
            auto key = std::make_tuple(point.x, point.y, point.z);
            if (unique_points.find(key) == unique_points.end()) {
                unique_points.insert(key);
                boundary_points.push_back(point);
            }
        };

        add_unique_point(*min_x_it);
        add_unique_point(*max_x_it);
        add_unique_point(*min_y_it);
        add_unique_point(*max_y_it);

        // If we have at least 3 unique points, create the hull
        if (boundary_points.size() >= 3)
        {
            // Sort points by angle around centroid for proper ordering
            pcl::PointXYZ centroid;
            centroid.x = centroid.y = centroid.z = 0;
            for (const auto& point : boundary_points)
            {
                centroid.x += point.x;
                centroid.y += point.y;
                centroid.z += point.z;
            }
            centroid.x /= boundary_points.size();
            centroid.y /= boundary_points.size();
            centroid.z /= boundary_points.size();

            // Sort by angle from centroid
            std::sort(boundary_points.begin(), boundary_points.end(),
                [&centroid](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                    float angle_a = std::atan2(a.y - centroid.y, a.x - centroid.x);
                    float angle_b = std::atan2(b.y - centroid.y, b.x - centroid.x);
                    return angle_a < angle_b;
                });

            hull->points = boundary_points;
        }
        else
        {
            // Fallback: use a subset of the original points
            size_t step = std::max(static_cast<size_t>(1), cluster->size() / 8);
            for (size_t i = 0; i < cluster->size() && hull->points.size() < 8; i += step)
            {
                hull->points.push_back(cluster->points[i]);
            }
        }

        hull->width = hull->points.size();
        hull->height = 1;
        hull->is_dense = true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error generating convex hull: %s", e.what());
    }

    return hull;
}

geometry_msgs::msg::Point SurfaceReconstructor::pclToGeometryPoint(const pcl::PointXYZ& pcl_point)
{
    geometry_msgs::msg::Point point;
    point.x = pcl_point.x;
    point.y = pcl_point.y;
    point.z = pcl_point.z;
    return point;
}

} // namespace map_builder