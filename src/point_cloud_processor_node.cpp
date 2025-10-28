#include "map_builder/point_cloud_processor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<map_builder::PointCloudProcessor>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in point cloud processor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}