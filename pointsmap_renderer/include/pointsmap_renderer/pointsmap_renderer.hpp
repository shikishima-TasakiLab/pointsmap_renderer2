#pragma once
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include "pointsmap_renderer/voxel_grid_map.hpp"
#include "vgm_msgs/msg/voxel_grid_map.hpp"

#define DEFAULT_HZ 5.0
#define DEFAULT_QUEUE_SIZE 10
#define DEFAULT_DEPTH_MIN 0.0
#define DEFAULT_DEPTH_MAX INFINITY

#define THROTTLE_PERIOD 30000

class Pointsmap_Renderer: public rclcpp::Node
{
public:
    Pointsmap_Renderer();
private:
    std::mutex _mtx;

    sensor_msgs::msg::CameraInfo::SharedPtr _camerainfo_msg;
    vgm_msgs::msg::VoxelGridMap::SharedPtr _map_msg;

    rclcpp::TimerBase::SharedPtr _timer;

    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _depth_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camerainfo_sub;
    rclcpp::Subscription<vgm_msgs::msg::VoxelGridMap>::SharedPtr _map_sub;

    Range _depth_range = {DEFAULT_DEPTH_MIN, DEFAULT_DEPTH_MAX};

    void _camerainfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void _map_callback(const vgm_msgs::msg::VoxelGridMap::SharedPtr msg);
    void _timer_callback();
};
