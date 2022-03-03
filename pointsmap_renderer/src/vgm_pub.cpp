#include "pointsmap_renderer/vgm_pub.hpp"

// コンストラクタ
VGM_Pub::VGM_Pub()
:   rclcpp::Node("vgm_pub")
{
    // voxel_sizeの設定を取得
    this->_voxel_size = static_cast<float>(this->declare_parameter("voxel_size", DEFAULT_VOXEL_SIZE));
    RCLCPP_INFO_STREAM(this->get_logger(), "voxel_size: " << this->_voxel_size);

    // Publisherの初期化
    this->_pub = this->create_publisher<vgm_msgs::msg::VoxelGridMap>("/voxel_grid_map", 2);

    // init_mapsの設定の取得
    std::vector<std::string> init_maps;
    init_maps = this->declare_parameter("init_maps", init_maps);
    // init_mapsの設定が存在する場合，地図を読み込んでPublish
    if (init_maps.size() > 0) {
        std::string map_frameid = DEFAULT_MAP_FRAMEID;
        map_frameid = this->declare_parameter("map_frameid", map_frameid);

        VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr> vgm(this->get_logger(), init_maps, map_frameid, this->_voxel_size);
        vgm_msgs::msg::VoxelGridMap::SharedPtr vgm_msg = vgm.get_vgm();
        this->_pub->publish(*vgm_msg);
    }

    // Subscriberの初期化
    this->_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points_map", rclcpp::QoS(2),
        std::bind(&VGM_Pub::_callback, this, std::placeholders::_1)
    );
}

// Subscriberのcallback関数
void VGM_Pub::_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 取得した三次元点群地図の読込
    VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr> vgm(this->get_logger(), msg, this->_voxel_size);
    vgm_msgs::msg::VoxelGridMap::SharedPtr vgm_msg = vgm.get_vgm();
    // VoxelGridMapのPublish
    this->_pub->publish(*vgm_msg);
}
