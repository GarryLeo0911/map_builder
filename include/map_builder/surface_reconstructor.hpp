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
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <memory>

namespace map_builder
{

class SurfaceReconstructor : public rclcpp::Node
{
public:
    SurfaceReconstructor();
    ~SurfaceReconstructor() = default;

private:
    // ROS2 parameters
    double mesh_resolution_;
    double clustering_tolerance_;
    int clustering_min_cluster_size_;
    int clustering_max_cluster_size_;
    double convex_hull_alpha_;

    // Point cloud processing
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;

    // ROS2 subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr surface_pub_;

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // Callback functions
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Processing functions
    std::vector<PointCloud::Ptr> clusterPoints(PointCloud::Ptr cloud);
    visualization_msgs::msg::MarkerArray generateMeshMarkers(
        const std::vector<PointCloud::Ptr>& clusters, 
        const std_msgs::msg::Header& header);
    visualization_msgs::msg::MarkerArray generateSurfaceMarkers(
        const std::vector<PointCloud::Ptr>& clusters, 
        const std_msgs::msg::Header& header);

    // Mesh generation functions
    std::vector<pcl::Vertices> generateTriangles(PointCloud::Ptr cluster);
    PointCloud::Ptr generateConvexHull(PointCloud::Ptr cluster);

    // Utility functions
    void declareParameters();
    void getParameters();
    geometry_msgs::msg::Point pclToGeometryPoint(const PointType& pcl_point);
};

} // namespace map_builder