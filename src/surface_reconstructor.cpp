#include "map_builder/surface_reconstructor.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>

// Enhanced surface reconstruction includes
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

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
    this->declare_parameter<double>("mesh_resolution", 0.05);
    this->declare_parameter<double>("clustering_tolerance", 0.1);
    this->declare_parameter<int>("clustering_min_cluster_size", 100);
    this->declare_parameter<int>("clustering_max_cluster_size", 50000);
    this->declare_parameter<double>("convex_hull_alpha", 0.1);
    
    // Enhanced surface reconstruction parameters
    this->declare_parameter<std::string>("reconstruction_method", "poisson"); // poisson, greedy_projection, marching_cubes
    this->declare_parameter<int>("poisson_depth", 8);
    this->declare_parameter<double>("poisson_width", 0.0);
    this->declare_parameter<double>("poisson_scale", 1.1);
    this->declare_parameter<int>("poisson_iso_divide", 8);
    this->declare_parameter<bool>("poisson_confidence", false);
    this->declare_parameter<bool>("poisson_output_polygons", false);
    
    // Greedy projection triangulation parameters
    this->declare_parameter<double>("gp3_search_radius", 0.2);
    this->declare_parameter<double>("gp3_mu", 2.5);
    this->declare_parameter<int>("gp3_max_nearest_neighbors", 100);
    this->declare_parameter<double>("gp3_max_surface_angle", M_PI/4);
    this->declare_parameter<double>("gp3_min_angle", M_PI/18);
    this->declare_parameter<double>("gp3_max_angle", 2*M_PI/3);
    
    // Normal estimation parameters
    this->declare_parameter<double>("normal_search_radius", 0.1);
    this->declare_parameter<int>("normal_k_search", 20);
    
    // Region growing parameters
    this->declare_parameter<bool>("enable_region_growing", true);
    this->declare_parameter<int>("region_growing_min_cluster_size", 50);
    this->declare_parameter<int>("region_growing_max_cluster_size", 1000000);
    this->declare_parameter<int>("region_growing_neighbors", 30);
    this->declare_parameter<double>("region_growing_smoothness_threshold", 3.0);
    this->declare_parameter<double>("region_growing_curvature_threshold", 1.0);
}

void SurfaceReconstructor::getParameters()
{
    this->get_parameter("mesh_resolution", mesh_resolution_);
    this->get_parameter("clustering_tolerance", clustering_tolerance_);
    this->get_parameter("clustering_min_cluster_size", clustering_min_cluster_size_);
    this->get_parameter("clustering_max_cluster_size", clustering_max_cluster_size_);
    this->get_parameter("convex_hull_alpha", convex_hull_alpha_);
    
    // Enhanced surface reconstruction parameters
    this->get_parameter("reconstruction_method", reconstruction_method_);
    this->get_parameter("poisson_depth", poisson_depth_);
    this->get_parameter("poisson_width", poisson_width_);
    this->get_parameter("poisson_scale", poisson_scale_);
    this->get_parameter("poisson_iso_divide", poisson_iso_divide_);
    this->get_parameter("poisson_confidence", poisson_confidence_);
    this->get_parameter("poisson_output_polygons", poisson_output_polygons_);
    
    // Greedy projection triangulation parameters
    this->get_parameter("gp3_search_radius", gp3_search_radius_);
    this->get_parameter("gp3_mu", gp3_mu_);
    this->get_parameter("gp3_max_nearest_neighbors", gp3_max_nearest_neighbors_);
    this->get_parameter("gp3_max_surface_angle", gp3_max_surface_angle_);
    this->get_parameter("gp3_min_angle", gp3_min_angle_);
    this->get_parameter("gp3_max_angle", gp3_max_angle_);
    
    // Normal estimation parameters
    this->get_parameter("normal_search_radius", normal_search_radius_);
    this->get_parameter("normal_k_search", normal_k_search_);
    
    // Region growing parameters
    this->get_parameter("enable_region_growing", enable_region_growing_);
    this->get_parameter("region_growing_min_cluster_size", region_growing_min_cluster_size_);
    this->get_parameter("region_growing_max_cluster_size", region_growing_max_cluster_size_);
    this->get_parameter("region_growing_neighbors", region_growing_neighbors_);
    this->get_parameter("region_growing_smoothness_threshold", region_growing_smoothness_threshold_);
    this->get_parameter("region_growing_curvature_threshold", region_growing_curvature_threshold_);

    RCLCPP_INFO(this->get_logger(), "Enhanced parameters loaded - method: %s, mesh resolution: %.3f, clustering tolerance: %.3f", 
                reconstruction_method_.c_str(), mesh_resolution_, clustering_tolerance_);
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

        // Estimate normals for the point cloud
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = estimateNormals(cloud);

        if (cloud_with_normals->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to estimate normals");
            return;
        }

        // Segment the point cloud using region growing or clustering
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> segments;
        if (enable_region_growing_)
        {
            segments = regionGrowingSegmentation(cloud_with_normals);
        }
        else
        {
            segments = euclideanClustering(cloud_with_normals);
        }

        if (segments.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No segments found for surface reconstruction");
            return;
        }

        // Generate enhanced mesh markers for each segment
        auto mesh_markers = generateEnhancedMeshMarkers(segments, msg->header);

        // Generate surface markers with improved visualization
        auto surface_markers = generateEnhancedSurfaceMarkers(segments, msg->header);

        // Publish markers
        if (!mesh_markers.markers.empty())
        {
            mesh_pub_->publish(mesh_markers);
        }

        if (!surface_markers.markers.empty())
        {
            surface_pub_->publish(surface_markers);
        }

        RCLCPP_DEBUG(this->get_logger(), "Generated enhanced surface reconstruction with %zu segments using %s method", 
                     segments.size(), reconstruction_method_.c_str());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in enhanced surface reconstruction: %s", e.what());
    }
}

pcl::PointCloud<pcl::PointNormal>::Ptr SurfaceReconstructor::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    try
    {
        if (cloud->empty())
        {
            return cloud_with_normals;
        }

        // Estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(normal_search_radius_);
        ne.setKSearch(normal_k_search_);
        ne.compute(*normals);

        // Combine points and normals
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        RCLCPP_DEBUG(this->get_logger(), "Estimated normals for %zu points", cloud_with_normals->size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error estimating normals: %s", e.what());
    }

    return cloud_with_normals;
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> SurfaceReconstructor::regionGrowingSegmentation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> segments;

    try
    {
        if (cloud_with_normals->empty())
        {
            return segments;
        }

        // Convert to XYZ for region growing
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_with_normals, *cloud_xyz);

        // Extract normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        for (const auto& point : cloud_with_normals->points)
        {
            pcl::Normal normal;
            normal.normal_x = point.normal_x;
            normal.normal_y = point.normal_y;
            normal.normal_z = point.normal_z;
            normals->points.push_back(normal);
        }
        normals->width = normals->points.size();
        normals->height = 1;

        // Region growing segmentation
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        
        rg.setMinClusterSize(region_growing_min_cluster_size_);
        rg.setMaxClusterSize(region_growing_max_cluster_size_);
        rg.setSearchMethod(tree);
        rg.setNumberOfNeighbours(region_growing_neighbors_);
        rg.setInputCloud(cloud_xyz);
        rg.setInputNormals(normals);
        rg.setSmoothnessThreshold(region_growing_smoothness_threshold_ / 180.0 * M_PI);
        rg.setCurvatureThreshold(region_growing_curvature_threshold_);

        std::vector<pcl::PointIndices> cluster_indices;
        rg.extract(cluster_indices);

        // Extract segments
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr segment(new pcl::PointCloud<pcl::PointNormal>);
            pcl::ExtractIndices<pcl::PointNormal> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));
            
            extract.setInputCloud(cloud_with_normals);
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*segment);

            if (!segment->empty())
            {
                segments.push_back(segment);
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Region growing found %zu segments", segments.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in region growing segmentation: %s", e.what());
    }

    return segments;
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> SurfaceReconstructor::euclideanClustering(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> segments;

    try
    {
        if (cloud_with_normals->empty())
        {
            return segments;
        }

        // Convert to XYZ for clustering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_with_normals, *cloud_xyz);

        // Euclidean cluster extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_xyz);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(clustering_tolerance_);
        ec.setMinClusterSize(clustering_min_cluster_size_);
        ec.setMaxClusterSize(clustering_max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_xyz);
        ec.extract(cluster_indices);

        // Extract segments
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr segment(new pcl::PointCloud<pcl::PointNormal>);
            pcl::ExtractIndices<pcl::PointNormal> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));
            
            extract.setInputCloud(cloud_with_normals);
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*segment);

            if (!segment->empty())
            {
                segments.push_back(segment);
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Euclidean clustering found %zu segments", segments.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in euclidean clustering: %s", e.what());
    }

    return segments;
}

visualization_msgs::msg::MarkerArray SurfaceReconstructor::generateEnhancedMeshMarkers(
    const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& segments, 
    const std_msgs::msg::Header& header)
{
    visualization_msgs::msg::MarkerArray marker_array;

    try
    {
        for (size_t i = 0; i < segments.size(); ++i)
        {
            const auto& segment = segments[i];

            if (segment->size() < 10) // Need sufficient points for mesh
            {
                continue;
            }

            // Generate mesh for this segment
            pcl::PolygonMesh mesh;
            if (!generateMesh(segment, mesh))
            {
                continue;
            }

            // Create mesh marker
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "enhanced_mesh_surfaces";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set marker properties
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            marker.color.r = 0.0;
            marker.color.g = 0.8;
            marker.color.b = 0.2;
            marker.color.a = 0.7;

            // Convert mesh to marker
            convertMeshToMarker(mesh, marker);

            if (!marker.points.empty())
            {
                marker_array.markers.push_back(marker);
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error generating enhanced mesh markers: %s", e.what());
    }

    return marker_array;
}

visualization_msgs::msg::MarkerArray SurfaceReconstructor::generateEnhancedSurfaceMarkers(
    const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& segments, 
    const std_msgs::msg::Header& header)
{
    visualization_msgs::msg::MarkerArray marker_array;

    try
    {
        for (size_t i = 0; i < segments.size(); ++i)
        {
            const auto& segment = segments[i];

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "enhanced_surface_points";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set marker properties
            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;

            // Color based on segment index
            double hue = (static_cast<double>(i) / segments.size()) * 360.0;
            marker.color.r = (hue < 120.0) ? (120.0 - hue) / 120.0 : (hue > 240.0) ? (hue - 240.0) / 120.0 : 0.0;
            marker.color.g = (hue < 240.0) ? std::max(0.0, (120.0 - std::abs(hue - 120.0)) / 120.0) : 0.0;
            marker.color.b = (hue > 120.0) ? std::max(0.0, (120.0 - std::abs(hue - 240.0)) / 120.0) : 0.0;
            marker.color.a = 0.8;

            // Add points (subsample for performance)
            size_t step = std::max(static_cast<size_t>(1), segment->size() / 200);
            for (size_t j = 0; j < segment->size(); j += step)
            {
                geometry_msgs::msg::Point point;
                point.x = segment->points[j].x;
                point.y = segment->points[j].y;
                point.z = segment->points[j].z;
                marker.points.push_back(point);
            }

            if (!marker.points.empty())
            {
                marker_array.markers.push_back(marker);
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error generating enhanced surface markers: %s", e.what());
    }

    return marker_array;
}

bool SurfaceReconstructor::generateMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        if (reconstruction_method_ == "poisson")
        {
            return generatePoissonMesh(segment, mesh);
        }
        else if (reconstruction_method_ == "greedy_projection")
        {
            return generateGreedyProjectionMesh(segment, mesh);
        }
        else if (reconstruction_method_ == "marching_cubes")
        {
            return generateMarchingCubesMesh(segment, mesh);
        }
        else
        {
            return generateSimpleTriangulation(segment, mesh);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error generating mesh: %s", e.what());
        return false;
    }
}

bool SurfaceReconstructor::generatePoissonMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        if (segment->size() < 10)
        {
            return false;
        }

        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setInputCloud(segment);
        poisson.setDepth(poisson_depth_);
        poisson.setScale(poisson_scale_);
        poisson.setIsoDivide(poisson_iso_divide_);
        poisson.setConfidence(poisson_confidence_);
        poisson.setOutputPolygons(poisson_output_polygons_);

        poisson.performReconstruction(mesh);

        return !mesh.polygons.empty();
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error in Poisson reconstruction: %s", e.what());
        return false;
    }
}

bool SurfaceReconstructor::generateGreedyProjectionMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        if (segment->size() < 10)
        {
            return false;
        }

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);

        gp3.setSearchMethod(tree);
        gp3.setInputCloud(segment);
        gp3.setSearchRadius(gp3_search_radius_);
        gp3.setMu(gp3_mu_);
        gp3.setMaximumNearestNeighbors(gp3_max_nearest_neighbors_);
        gp3.setMaximumSurfaceAngle(gp3_max_surface_angle_);
        gp3.setMinimumAngle(gp3_min_angle_);
        gp3.setMaximumAngle(gp3_max_angle_);
        gp3.setNormalConsistency(false);

        gp3.reconstruct(mesh);

        return !mesh.polygons.empty();
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error in greedy projection reconstruction: %s", e.what());
        return false;
    }
}

bool SurfaceReconstructor::generateMarchingCubesMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        // Marching cubes might not be available in this PCL version
        // Fall back to simple triangulation
        RCLCPP_DEBUG(this->get_logger(), "Marching cubes not available, using simple triangulation");
        return generateSimpleTriangulation(segment, mesh);
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error in marching cubes reconstruction: %s", e.what());
        return false;
    }
}

bool SurfaceReconstructor::generateSimpleTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        if (segment->size() < 4)
        {
            return false;
        }

        // Convert to XYZ for convex hull
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*segment, *cloud_xyz);

        // Generate convex hull
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(cloud_xyz);
        hull.reconstruct(mesh);

        return !mesh.polygons.empty();
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error in simple triangulation: %s", e.what());
        return false;
    }
}

void SurfaceReconstructor::convertMeshToMarker(const pcl::PolygonMesh& mesh, visualization_msgs::msg::Marker& marker)
{
    try
    {
        // Convert PCL mesh to ROS marker
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);

        for (const auto& polygon : mesh.polygons)
        {
            if (polygon.vertices.size() >= 3)
            {
                // Create triangles from polygon
                for (size_t i = 1; i < polygon.vertices.size() - 1; ++i)
                {
                    marker.points.push_back(pclToGeometryPoint(cloud.points[polygon.vertices[0]]));
                    marker.points.push_back(pclToGeometryPoint(cloud.points[polygon.vertices[i]]));
                    marker.points.push_back(pclToGeometryPoint(cloud.points[polygon.vertices[i + 1]]));
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error converting mesh to marker: %s", e.what());
    }
}

double SurfaceReconstructor::calculateAveragePointDistance(pcl::PointCloud<pcl::PointNormal>::Ptr segment)
{
    if (segment->size() < 2)
    {
        return 0.1; // Default distance
    }

    double total_distance = 0.0;
    int count = 0;

    // Sample a subset of points for efficiency
    size_t step = std::max(static_cast<size_t>(1), segment->size() / 50);
    for (size_t i = 0; i < segment->size(); i += step)
    {
        for (size_t j = i + step; j < segment->size() && count < 100; j += step)
        {
            double dx = segment->points[i].x - segment->points[j].x;
            double dy = segment->points[i].y - segment->points[j].y;
            double dz = segment->points[i].z - segment->points[j].z;
            total_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
            count++;
        }
    }

    return count > 0 ? total_distance / count : 0.1;
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
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> boundary_points;
        
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

pcl::PointCloud<pcl::PointNormal>::Ptr SurfaceReconstructor::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    try
    {
        // Copy points
        cloud_with_normals->resize(cloud->size());
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            cloud_with_normals->points[i].x = cloud->points[i].x;
            cloud_with_normals->points[i].y = cloud->points[i].y;
            cloud_with_normals->points[i].z = cloud->points[i].z;
        }

        // Estimate normals using OpenMP for performance
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        
        if (normal_search_radius_ > 0)
        {
            ne.setRadiusSearch(normal_search_radius_);
        }
        else
        {
            ne.setKSearch(normal_k_search_);
        }

        ne.compute(*normals);

        // Copy normals
        for (size_t i = 0; i < normals->size(); ++i)
        {
            cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
            cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
            cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;
            cloud_with_normals->points[i].curvature = normals->points[i].curvature;
        }

        cloud_with_normals->width = cloud_with_normals->points.size();
        cloud_with_normals->height = 1;
        cloud_with_normals->is_dense = false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error estimating normals: %s", e.what());
        return cloud_with_normals;
    }

    return cloud_with_normals;
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> SurfaceReconstructor::regionGrowingSegmentation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> segments;

    try
    {
        // Convert PointNormal to PointXYZ for region growing
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        cloud->resize(cloud_with_normals->size());
        normals->resize(cloud_with_normals->size());

        for (size_t i = 0; i < cloud_with_normals->size(); ++i)
        {
            cloud->points[i].x = cloud_with_normals->points[i].x;
            cloud->points[i].y = cloud_with_normals->points[i].y;
            cloud->points[i].z = cloud_with_normals->points[i].z;

            normals->points[i].normal_x = cloud_with_normals->points[i].normal_x;
            normals->points[i].normal_y = cloud_with_normals->points[i].normal_y;
            normals->points[i].normal_z = cloud_with_normals->points[i].normal_z;
            normals->points[i].curvature = cloud_with_normals->points[i].curvature;
        }

        // Setup region growing
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        reg.setMinClusterSize(region_growing_min_cluster_size_);
        reg.setMaxClusterSize(region_growing_max_cluster_size_);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(region_growing_neighbors_);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(region_growing_smoothness_threshold_ / 180.0 * M_PI);
        reg.setCurvatureThreshold(region_growing_curvature_threshold_);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // Extract segments
        for (const auto& indices : clusters)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr segment(new pcl::PointCloud<pcl::PointNormal>);
            pcl::ExtractIndices<pcl::PointNormal> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));

            extract.setInputCloud(cloud_with_normals);
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*segment);

            if (!segment->empty())
            {
                segments.push_back(segment);
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Region growing found %zu segments", segments.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in region growing segmentation: %s", e.what());
    }

    return segments;
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> SurfaceReconstructor::euclideanClustering(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> segments;

    try
    {
        // Convert to PointXYZ for clustering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(cloud_with_normals->size());

        for (size_t i = 0; i < cloud_with_normals->size(); ++i)
        {
            cloud->points[i].x = cloud_with_normals->points[i].x;
            cloud->points[i].y = cloud_with_normals->points[i].y;
            cloud->points[i].z = cloud_with_normals->points[i].z;
        }

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

        // Extract segments
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr segment(new pcl::PointCloud<pcl::PointNormal>);
            pcl::ExtractIndices<pcl::PointNormal> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));

            extract.setInputCloud(cloud_with_normals);
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*segment);

            if (!segment->empty())
            {
                segments.push_back(segment);
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Euclidean clustering found %zu segments", segments.size());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in euclidean clustering: %s", e.what());
    }

    return segments;
}

visualization_msgs::msg::MarkerArray SurfaceReconstructor::generateEnhancedMeshMarkers(
    const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& segments, 
    const std_msgs::msg::Header& header)
{
    visualization_msgs::msg::MarkerArray marker_array;

    try
    {
        for (size_t i = 0; i < segments.size(); ++i)
        {
            const auto& segment = segments[i];

            if (segment->size() < 4) // Need at least 4 points for mesh
            {
                continue;
            }

            // Create mesh marker
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "enhanced_mesh_surfaces";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set marker properties
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            // Color based on segment size and height
            double avg_height = 0.0;
            for (const auto& point : segment->points)
            {
                avg_height += point.z;
            }
            avg_height /= segment->points.size();

            // Color coding: blue for ground, green for mid-level, red for high
            double normalized_height = std::max(0.0, std::min(1.0, (avg_height + 1.0) / 3.0));
            marker.color.r = normalized_height;
            marker.color.g = 0.8 * (1.0 - std::abs(normalized_height - 0.5) * 2);
            marker.color.b = 1.0 - normalized_height;
            marker.color.a = 0.7;

            // Generate mesh based on selected method
            pcl::PolygonMesh mesh;
            if (generateMesh(segment, mesh))
            {
                // Convert mesh to marker triangles
                convertMeshToMarker(mesh, marker);
            }

            if (!marker.points.empty())
            {
                marker_array.markers.push_back(marker);
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error generating enhanced mesh markers: %s", e.what());
    }

    return marker_array;
}

bool SurfaceReconstructor::generateMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        if (reconstruction_method_ == "poisson")
        {
            return generatePoissonMesh(segment, mesh);
        }
        else if (reconstruction_method_ == "greedy_projection")
        {
            return generateGreedyProjectionMesh(segment, mesh);
        }
        else if (reconstruction_method_ == "marching_cubes")
        {
            return generateMarchingCubesMesh(segment, mesh);
        }
        else
        {
            // Fallback to simple triangulation
            return generateSimpleTriangulation(segment, mesh);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error generating mesh with method %s: %s", 
                     reconstruction_method_.c_str(), e.what());
        return false;
    }
}

bool SurfaceReconstructor::generatePoissonMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setInputCloud(segment);
        poisson.setDepth(poisson_depth_);
        poisson.setWidth(poisson_width_);
        poisson.setScale(poisson_scale_);
        poisson.setIsoDivide(poisson_iso_divide_);
        poisson.setConfidence(poisson_confidence_);
        poisson.setOutputPolygons(poisson_output_polygons_);

        poisson.reconstruct(mesh);
        return !mesh.polygons.empty();
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Poisson reconstruction failed: %s", e.what());
        return false;
    }
}

bool SurfaceReconstructor::generateGreedyProjectionMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        // Setup search tree
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(segment);

        // Setup greedy projection triangulation
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        gp3.setSearchRadius(gp3_search_radius_);
        gp3.setMu(gp3_mu_);
        gp3.setMaximumNearestNeighbors(gp3_max_nearest_neighbors_);
        gp3.setMaximumSurfaceAngle(gp3_max_surface_angle_);
        gp3.setMinimumAngle(gp3_min_angle_);
        gp3.setMaximumAngle(gp3_max_angle_);
        gp3.setNormalConsistency(false);

        gp3.setInputCloud(segment);
        gp3.setSearchMethod(tree);
        gp3.reconstruct(mesh);

        return !mesh.polygons.empty();
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Greedy projection triangulation failed: %s", e.what());
        return false;
    }
}

bool SurfaceReconstructor::generateMarchingCubesMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    try
    {
        pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
        mc.setInputCloud(segment);
        mc.setGridResolution(static_cast<int>(1.0 / mesh_resolution_));
        mc.setIsoLevel(0.0);

        mc.reconstruct(mesh);
        return !mesh.polygons.empty();
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Marching cubes reconstruction failed: %s", e.what());
        return false;
    }
}

bool SurfaceReconstructor::generateSimpleTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh)
{
    // Simple fallback: create triangles from convex hull
    try
    {
        // Convert to PointXYZ for hull generation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(segment->size());

        for (size_t i = 0; i < segment->size(); ++i)
        {
            cloud->points[i].x = segment->points[i].x;
            cloud->points[i].y = segment->points[i].y;
            cloud->points[i].z = segment->points[i].z;
        }

        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(cloud);
        hull.reconstruct(mesh);

        return !mesh.polygons.empty();
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Simple triangulation failed: %s", e.what());
        return false;
    }
}

void SurfaceReconstructor::convertMeshToMarker(const pcl::PolygonMesh& mesh, visualization_msgs::msg::Marker& marker)
{
    try
    {
        // Convert PCL mesh to ROS marker
        pcl::PointCloud<pcl::PointXYZ> vertices;
        pcl::fromPCLPointCloud2(mesh.cloud, vertices);

        for (const auto& polygon : mesh.polygons)
        {
            if (polygon.vertices.size() >= 3)
            {
                // Create triangles (fan triangulation for polygons with >3 vertices)
                for (size_t i = 1; i < polygon.vertices.size() - 1; ++i)
                {
                    uint32_t v0 = polygon.vertices[0];
                    uint32_t v1 = polygon.vertices[i];
                    uint32_t v2 = polygon.vertices[i + 1];

                    if (v0 < vertices.size() && v1 < vertices.size() && v2 < vertices.size())
                    {
                        marker.points.push_back(pclToGeometryPoint(vertices[v0]));
                        marker.points.push_back(pclToGeometryPoint(vertices[v1]));
                        marker.points.push_back(pclToGeometryPoint(vertices[v2]));
                    }
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error converting mesh to marker: %s", e.what());
    }
}

visualization_msgs::msg::MarkerArray SurfaceReconstructor::generateEnhancedSurfaceMarkers(
    const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& segments, 
    const std_msgs::msg::Header& header)
{
    visualization_msgs::msg::MarkerArray marker_array;

    try
    {
        for (size_t i = 0; i < segments.size(); ++i)
        {
            const auto& segment = segments[i];

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "enhanced_surface_points";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Adaptive marker size based on point cloud density
            double avg_distance = calculateAveragePointDistance(segment);
            marker.scale.x = std::max(0.02, std::min(0.1, avg_distance * 0.5));
            marker.scale.y = marker.scale.x;
            marker.scale.z = marker.scale.x;

            // Enhanced color coding based on surface normal and height
            double avg_height = 0.0;
            double avg_normal_z = 0.0;
            for (const auto& point : segment->points)
            {
                avg_height += point.z;
                avg_normal_z += point.normal_z;
            }
            avg_height /= segment->points.size();
            avg_normal_z /= segment->points.size();

            // Color based on surface orientation and height
            double surface_type = std::abs(avg_normal_z); // 1.0 = horizontal, 0.0 = vertical
            double normalized_height = std::max(0.0, std::min(1.0, (avg_height + 2.0) / 4.0));

            marker.color.r = normalized_height * 0.8 + surface_type * 0.2;
            marker.color.g = surface_type * 0.8 + (1.0 - normalized_height) * 0.2;
            marker.color.b = (1.0 - surface_type) * 0.8 + normalized_height * 0.2;
            marker.color.a = 0.8;

            // Subsample points for visualization
            size_t step = std::max(static_cast<size_t>(1), segment->size() / 200);
            for (size_t j = 0; j < segment->size(); j += step)
            {
                geometry_msgs::msg::Point point;
                point.x = segment->points[j].x;
                point.y = segment->points[j].y;
                point.z = segment->points[j].z;
                marker.points.push_back(point);
            }

            if (!marker.points.empty())
            {
                marker_array.markers.push_back(marker);
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error generating enhanced surface markers: %s", e.what());
    }

    return marker_array;
}

double SurfaceReconstructor::calculateAveragePointDistance(pcl::PointCloud<pcl::PointNormal>::Ptr segment)
{
    try
    {
        if (segment->size() < 2)
        {
            return 0.05; // Default value
        }

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(segment);

        double total_distance = 0.0;
        int count = 0;

        // Sample a subset of points to calculate average distance to nearest neighbor
        size_t step = std::max(static_cast<size_t>(1), segment->size() / 50);
        for (size_t i = 0; i < segment->size(); i += step)
        {
            std::vector<int> indices(2);
            std::vector<float> distances(2);

            if (tree->nearestKSearch(segment->points[i], 2, indices, distances) == 2)
            {
                total_distance += std::sqrt(distances[1]); // Distance to second nearest (first is itself)
                count++;
            }
        }

        return count > 0 ? total_distance / count : 0.05;
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Error calculating average point distance: %s", e.what());
        return 0.05;
    }
}

} // namespace map_builder