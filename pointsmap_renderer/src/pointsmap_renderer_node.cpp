#include "pointsmap_renderer/pointsmap_renderer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pointsmap_Renderer>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
