#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "vgm_msgs/msg/voxel_grid_map.hpp"
#include "voxel_grid_map.hpp"

#define DEFAULT_MAP_FRAMEID "map"

class VGM_Pub: public rclcpp::Node
{
public:
    VGM_Pub();
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;
    rclcpp::Publisher<vgm_msgs::msg::VoxelGridMap>::SharedPtr _pub;
    float_t _voxel_size = DEFAULT_VOXEL_SIZE;

    void _callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
