
#include "rclcpp/rclcpp.hpp"
#include <fstream>

#include "map_builder/surface_recon.hpp"
#include "map_builder/pset.hpp"
#include "map_builder/hole_filling.hpp"
#include "map_builder/rm_artifacts.hpp"
#include "map_builder/parser.hpp"
#include "map_builder/texture.hpp"

#include "map_builder/srv/normals_from_point_cloud2.hpp"

#include <map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <CGAL/Vector_3.h>

#include <CGAL/Kernel_traits.h>
#include <CGAL/boost/graph/properties.h>

#include <boost/cstdint.hpp>
#include <boost/graph/graph_traits.hpp>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <iostream>
#include <chrono>

using NormalsSrv = map_builder::srv::NormalsFromPointCloud2;

double gridm = 0.0;
double holemaxsize = 0.0;
std::string output_folder = "";
double sample = 0.0;
double trim = 0.0;
int texturesize = 0;

static rclcpp::Logger LOGGER = rclcpp::get_logger("map_builder");

std::string generate_point_cloud_normals(const sensor_msgs::msg::PointCloud2 & msg, const geometry_msgs::msg::Point & src, std::string output) {
    std::vector<Point> points;
    std::vector<Vector> normals;
    std::vector<Color> colors;
    std::list<PointVectorPair> estimated_pwn;
    std::vector<Point_3> orig_points;
    std::vector<unsigned int> indices;
    auto src_point = Point_3(src.x, src.y, src.z);

    FT average_spacing = 0.;

    auto start_total_time = chrono::high_resolution_clock::now();

    auto start = chrono::high_resolution_clock::now();
    Pset pointset(points, normals, colors, sample); // init my point cloud
    pointset.read_pointCloud2(msg);

    for (size_t i = 0; i < points.size(); i++) //initialize structure for Kdtree
    {
        orig_points.push_back(Point_3(points[i][0], points[i][1], points[i][2]));
        indices.push_back(i);
    }
    auto stop = chrono::high_resolution_clock::now();

    RCLCPP_INFO_STREAM(LOGGER, "Time initialize pointcloud: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    // Initialize a spatial kd-tree for further search on original set of points
    start = chrono::high_resolution_clock::now();
    Tree tree(
            boost::make_zip_iterator(boost::make_tuple(orig_points.begin(), indices.begin())),
            boost::make_zip_iterator(boost::make_tuple(orig_points.end(), indices.end()))
    );

    pointset.write_ply(output + "_full_pcd_rgb.ply"); //write full rgb .ply
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to generate full rgb *.ply file: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    start = chrono::high_resolution_clock::now();
    average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(orig_points.begin(), orig_points.end(), 6);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to compute_average_spacing: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    double as_ratio = gridm * log(points.size() / 10e4);

    if (as_ratio < 0.6)
        as_ratio = 0.6;

    double cell_size = average_spacing * as_ratio;

    RCLCPP_INFO_STREAM(LOGGER, "Estimated weight for grid size w.r.t. avg. point distance: " << as_ratio);

    start = chrono::high_resolution_clock::now();
    pointset.sample_points_cgal(cell_size, 1);
    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to sample_points_cgal: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    RCLCPP_INFO_STREAM(LOGGER,
            "Sampled point set size: " << points.size() << " | Normals: " << normals.size() << " | Original ply size: "
                                       << colors.size());

    start = chrono::high_resolution_clock::now();
    if (normals.size() == 0){
        RCLCPP_INFO_STREAM(LOGGER, "Estimating normals");
        estimate_normals(points, src_point, estimated_pwn);
    }
    else{
        RCLCPP_INFO_STREAM(LOGGER, "Registering normals normals");
        estimated_pwn = register_normals(points, grab_normals(orig_points, normals));
    }

    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to estimate normals: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");

    RCLCPP_INFO_STREAM(LOGGER, "Points sizes:" << points.size() << " Estimated PWN size:" << estimated_pwn.size());

    start = chrono::high_resolution_clock::now();
    string output_ply_filepath = output + "_simplified_pcd_normals.ply";
    //write_ply_wnormals(output_ply_filepath, estimated_pwn, tree, colors);
    write_ply_binary_wnormals(output_ply_filepath, estimated_pwn, tree, colors);

    stop = chrono::high_resolution_clock::now();
    RCLCPP_INFO_STREAM(LOGGER, "Time to write point cloud .ply file: "
                            << float(chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0)
                            << " seconds");
    RCLCPP_INFO_STREAM(LOGGER, "PLY file written to:" << output_ply_filepath);

    return output_ply_filepath;
}

// dynamic reconfigure removed for ROS2 migration; use node parameters instead

void pointNormalsFromPointCloudCallback(const std::shared_ptr<NormalsSrv::Request> req, std::shared_ptr<NormalsSrv::Response> res)
{
    RCLCPP_INFO(LOGGER, "pointNormalsFromPointCloudCallback called");

    try {
        const sensor_msgs::msg::PointCloud2 ptcloud = req->input;
        const geometry_msgs::msg::Point src = req->src;

        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, sizeof(buffer), "%d-%m-%Y_%H:%M:%S", timeinfo);

        std::string output_file = output_folder;
        char endch = output_file.back();
        if ('/' != endch && '\\' != endch) {
            output_file += "/";
        }
        output_file += buffer;

        RCLCPP_INFO_STREAM(LOGGER, "Output file:" << output_file);

        std::string output_path = generate_point_cloud_normals(ptcloud, src, output_file);

        res->success = true;
        res->path = output_path;
    } catch (std::runtime_error &e) {
        RCLCPP_ERROR(LOGGER, "genMeshFromPointCloudCallback exception: %s", e.what());
        res->success = false;
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("p_normal_service_node");

    RCLCPP_INFO(node->get_logger(), "Start p_normal_service node");

    std::string version(std::to_string(CGAL_VERSION_NR));
    RCLCPP_INFO_STREAM(node->get_logger(), "Using CGAL lib version " << version.substr(1, 2) << "." << version.substr(3, 2));

    node->declare_parameter<double>("gridm", gridm);
    node->declare_parameter<double>("holemaxsize", holemaxsize);
    node->declare_parameter<std::string>("output_folder", output_folder);
    node->declare_parameter<double>("sample", sample);
    node->declare_parameter<double>("trim", trim);
    node->declare_parameter<int>("texturesize", texturesize);

    auto service = node->create_service<NormalsSrv>("/pnormal_from_pointclouds", &pointNormalsFromPointCloudCallback);

    RCLCPP_INFO(node->get_logger(), "Spinning p_normal_service node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "p_normal_service node stopped");

    return 0;
}
