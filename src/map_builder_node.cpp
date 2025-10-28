#include "map_builder/map_builder.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<map_builder::MapBuilder>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in map builder: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}