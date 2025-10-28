#include "map_builder/surface_reconstructor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<map_builder::SurfaceReconstructor>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in surface reconstructor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}