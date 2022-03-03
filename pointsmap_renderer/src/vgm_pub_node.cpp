#include "pointsmap_renderer/vgm_pub.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VGM_Pub>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
