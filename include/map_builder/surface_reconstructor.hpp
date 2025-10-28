#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <memory>
#include <string>

namespace map_builder
{

class SurfaceReconstructor : public rclcpp::Node
{
public:
    // Point cloud processing
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;
    using PointNormalType = pcl::PointNormal;
    using PointNormalCloud = pcl::PointCloud<PointNormalType>;
    
    SurfaceReconstructor();
    ~SurfaceReconstructor() = default;

private:
    // Basic ROS2 parameters
    double mesh_resolution_;
    double clustering_tolerance_;
    int clustering_min_cluster_size_;
    int clustering_max_cluster_size_;
    double convex_hull_alpha_;
    
    // Enhanced surface reconstruction parameters
    std::string reconstruction_method_;
    int poisson_depth_;
    double poisson_width_;
    double poisson_scale_;
    int poisson_iso_divide_;
    bool poisson_confidence_;
    bool poisson_output_polygons_;
    
    // Greedy projection triangulation parameters
    double gp3_search_radius_;
    double gp3_mu_;
    int gp3_max_nearest_neighbors_;
    double gp3_max_surface_angle_;
    double gp3_min_angle_;
    double gp3_max_angle_;
    
    // Normal estimation parameters
    double normal_search_radius_;
    int normal_k_search_;
    
    // Region growing parameters
    bool enable_region_growing_;
    int region_growing_min_cluster_size_;
    int region_growing_max_cluster_size_;
    int region_growing_neighbors_;
    double region_growing_smoothness_threshold_;
    double region_growing_curvature_threshold_;

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surface_pub_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Callback functions
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Enhanced processing functions
    pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> regionGrowingSegmentation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> euclideanClustering(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
    
    visualization_msgs::msg::MarkerArray generateEnhancedMeshMarkers(
        const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& segments, 
        const std_msgs::msg::Header& header);
    visualization_msgs::msg::MarkerArray generateEnhancedSurfaceMarkers(
        const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& segments, 
        const std_msgs::msg::Header& header);

    // Mesh generation functions
    bool generateMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh);
    bool generatePoissonMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh);
    bool generateGreedyProjectionMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh);
    bool generateMarchingCubesMesh(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh);
    bool generateSimpleTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr segment, pcl::PolygonMesh& mesh);
    void convertMeshToMarker(const pcl::PolygonMesh& mesh, visualization_msgs::msg::Marker& marker);

    // Legacy functions (kept for compatibility)
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    visualization_msgs::msg::MarkerArray generateMeshMarkers(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, 
        const std_msgs::msg::Header& header);
    visualization_msgs::msg::MarkerArray generateSurfaceMarkers(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters, 
        const std_msgs::msg::Header& header);
    pcl::PointCloud<pcl::PointXYZ>::Ptr generateConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);

    // Utility functions
    void declareParameters();
    void getParameters();
    geometry_msgs::msg::Point pclToGeometryPoint(const pcl::PointXYZ& pcl_point);
    double calculateAveragePointDistance(pcl::PointCloud<pcl::PointNormal>::Ptr segment);
};

} // namespace map_builder