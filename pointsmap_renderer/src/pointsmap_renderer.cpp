#include "pointsmap_renderer/pointsmap_renderer.hpp"

Pointsmap_Renderer::Pointsmap_Renderer()
: rclcpp::Node("pointsmap_renderer")
{
    double_t hz = DEFAULT_HZ;
    hz = this->declare_parameter("hz", hz);
    RCLCPP_INFO_STREAM(this->get_logger(), "hz: " << hz);

    int queue_size = DEFAULT_QUEUE_SIZE;
    queue_size = this->declare_parameter("queue_size", queue_size);
    RCLCPP_INFO_STREAM(this->get_logger(), "queue_size: " << queue_size);

    this->_depth_range.min = static_cast<float>(this->declare_parameter("depth_min", DEFAULT_DEPTH_MIN));
    RCLCPP_INFO_STREAM(this->get_logger(), "depth_min: " << this->_depth_range.min);

    this->_depth_range.max = static_cast<float>(this->declare_parameter("depth_max", DEFAULT_DEPTH_MAX));
    RCLCPP_INFO_STREAM(this->get_logger(), "depth_max: " << this->_depth_range.max);

    this->_tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
    this->_tf_listener.reset(new tf2_ros::TransformListener(*this->_tf_buffer));

    this->_depth_pub = this->create_publisher<sensor_msgs::msg::Image>("sparse_depth", queue_size);

    this->_camerainfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::QoS(2),
        std::bind(&Pointsmap_Renderer::_camerainfo_callback, this, std::placeholders::_1)
    );
    this->_map_sub = this->create_subscription<vgm_msgs::msg::VoxelGridMap>(
        "/voxel_grid_map", rclcpp::QoS(2),
        std::bind(&Pointsmap_Renderer::_map_callback, this, std::placeholders::_1)
    );

    this->_timer = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int>(1e9 / hz)),
        std::bind(&Pointsmap_Renderer::_timer_callback, this)
    );
}

void Pointsmap_Renderer::_camerainfo_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(this->_mtx);
    this->_camerainfo_msg = msg;
}

void Pointsmap_Renderer::_map_callback(
    const vgm_msgs::msg::VoxelGridMap::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(this->_mtx);
    this->_map_msg = msg;
}

void Pointsmap_Renderer::_timer_callback()
{
    vgm_msgs::msg::VoxelGridMap::SharedPtr map_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camerainfo_msg;
    {
        std::lock_guard<std::mutex> lock(this->_mtx);
        map_msg = this->_map_msg;
        camerainfo_msg = this->_camerainfo_msg;
    }
    if (map_msg == nullptr) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD, "No points.");
        return;
    }
    if (map_msg->header.frame_id == "") {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD, "No frame_id (map).");
        return;
    }
    if (camerainfo_msg == nullptr) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD, "No camera_info.");
        return;
    }
    if (camerainfo_msg->header.frame_id == "") {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD, "No frame_id (camera_info)");
        return;
    }

    geometry_msgs::msg::TransformStamped tf_map2camera;
    try {
        tf_map2camera = this->_tf_buffer->lookupTransform(
            map_msg->header.frame_id,
            camerainfo_msg->header.frame_id,
            tf2::TimePoint()
        );
    }
    catch (tf2::TransformException &e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD, "[map(" << map_msg->header.frame_id << ") -> camera (" << camerainfo_msg->header.frame_id << ")] Transform Error: " << e.what());
        return;
    }

    VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr> vgm(this->get_logger(), map_msg);

    cv_bridge::CvImage depth;
    vgm.create_depthmap(camerainfo_msg, tf_map2camera, depth.image, this->_depth_range);
    sensor_msgs::msg::Image sparse_depth = *depth.toImageMsg();
    sparse_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sparse_depth.header.stamp = this->get_clock()->now();
    sparse_depth.header.frame_id = camerainfo_msg->header.frame_id;
    this->_depth_pub->publish(sparse_depth);
}
