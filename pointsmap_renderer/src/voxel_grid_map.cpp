#include "pointsmap_renderer/voxel_grid_map.hpp"

template <class VoxelGridMapPtrT>
VoxelGridMap<VoxelGridMapPtrT>::VoxelGridMap(
    const rclcpp::Logger &logger
) {
    this->_logger = std::make_shared<rclcpp::Logger>(logger);
}

template <class VoxelGridMapPtrT>
VoxelGridMap<VoxelGridMapPtrT>::VoxelGridMap(
    const rclcpp::Logger &logger,
    const VoxelGridMapPtrT &vgm
) {
    this->_voxel_grid_map = vgm;
    this->_logger = std::make_shared<rclcpp::Logger>(logger);
}

template <class VoxelGridMapPtrT>
bool VoxelGridMap<VoxelGridMapPtrT>::is_empty()
{
    return this->_voxel_grid_map->z.size() == 0UL;
}

template <class VoxelGridMapPtrT>
template <typename CameraInfoPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::create_depthmap(
    const CameraInfoPtrT &camerainfo,
    const geometry_msgs::msg::TransformStamped &tf,
    cv::Mat &depthmap,
    const Range &depth_range
) {
    CameraParam camera_param = {
        static_cast<float_t>(camerainfo->k[K::Fx]),
        static_cast<float_t>(camerainfo->k[K::Fy]),
        static_cast<float_t>(camerainfo->k[K::Cx]),
        static_cast<float_t>(camerainfo->k[K::Cy]),
        static_cast<int>(camerainfo->height),
        static_cast<int>(camerainfo->width)
    };
    XY<float_t> shape_f = {
        static_cast<float_t>(camera_param.width),
        static_cast<float_t>(camera_param.height)
    };
    LRTB<float_t> tan = {
        -camera_param.Cx / camera_param.Fx,
        (shape_f.x - camera_param.Cx) / camera_param.Fx,
        -camera_param.Cy / camera_param.Fy,
        (shape_f.y - camera_param.Cy) / camera_param.Fy
    };

    Eigen::Isometry3d tmp_tf = tf2::transformToEigen(tf);
    Eigen::Vector3f tr = tmp_tf.translation().cast<float_t>();
    Eigen::Quaternionf q(tmp_tf.rotation().cast<float_t>());

    this->_invert_transform(tr, q, tr, q);

    std::vector<XYZ<size_t> > voxel_idxs;
    this->_voxels_include_frustum(tr, q, depth_range, tan, voxel_idxs);
    this->_create_depthmap(voxel_idxs, tr, q, camera_param, shape_f, tan, depth_range, depthmap);
}

template <class VoxelGridMapPtrT>
VoxelGridMapPtrT VoxelGridMap<VoxelGridMapPtrT>::get_vgm()
{
    return this->_voxel_grid_map;
}

template <class VoxelGridMapPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::_invert_transform(
    const Eigen::Vector3f &in_tr,
    const Eigen::Quaternionf &in_q,
    Eigen::Vector3f &out_tr,
    Eigen::Quaternionf &out_q
) {
    Eigen::Vector3f translation;
    Eigen::Quaternionf quaternion;

    quaternion = in_q.conjugate();
    translation = -(quaternion * in_tr);

    out_tr = translation;
    out_q = quaternion;
}

template <class VoxelGridMapPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::_transform_pointcloud(
    const pcl::PointCloud<pcl::PointXYZL> &src,
    pcl::PointCloud<pcl::PointXYZL> &dst,
    const Eigen::Vector3f &translation,
    const Eigen::Quaternionf &quaternion
) {
    size_t len = src.points.size();

    if (len == 0ul) {
        dst = src;
        return;
    }

    if (
        translation == Eigen::Vector3f::Zero() &&
        quaternion.x() == 0.0f &&
        quaternion.y() == 0.0f &&
        quaternion.z() == 0.0f &&
        quaternion.w() == 1.0f
    ) {
        dst = src;
        return;
    }

    if (&src != &dst) {
        dst.points.resize(len);
        dst.width = src.width;
        dst.height = src.height;
        dst.is_dense = src.is_dense;
        dst.header = src.header;
    }

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif
    for (size_t i = 0ul; i < len; i++) {
        const pcl::PointXYZL *src_point_pcl = &src.points[i];
        Eigen::Vector3f src_point_eigen(src_point_pcl->x, src_point_pcl->y, src_point_pcl->z);
        Eigen::Vector3f dst_point_eigen = quaternion * src_point_eigen + translation;

        dst.points[i].x = dst_point_eigen[Axis::X];
        dst.points[i].y = dst_point_eigen[Axis::Y];
        dst.points[i].z = dst_point_eigen[Axis::Z];
        dst.points[i].label = src_point_pcl->label;
    }
}

template <class VoxelGridMapPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::_depth_filter(
    const pcl::PointCloud<pcl::PointXYZL> &src,
    pcl::PointCloud<pcl::PointXYZL> &dst,
    const Range &depth_range
) {
    size_t len = src.points.size();

    if (depth_range.min < 0.0F || depth_range.min == NAN)
        throw std::runtime_error("\"depth_range.min\" must be longer than 0.");
    if (depth_range.max <= 0.0F || depth_range.max == NAN)
        throw std::runtime_error("\"depth_range.max\" must be longer than 0.");
    if (len == 0UL) {
        dst = src;
        return;
    }

    if (&src != &dst) {
        dst.points.resize(len);
        dst.width = src.width;
        dst.height = src.height;
        dst.is_dense = src.is_dense;
        dst.header = src.header;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZL> extract;

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif
    for (size_t i = 0ul; i < len; i++) {
        if (src.points[i].z < depth_range.min || depth_range.max < src.points[i].z) {
            #ifdef _OPENMP
                #pragma omp critical
            #endif
            inliers->indices.push_back(i);
        }
    }

    extract.setInputCloud(dst.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(dst);
}

template <>
void VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>::_set_voxel_grid(
    const pcl::PointCloud<pcl::PointXYZL> &pointsmap,
    const std::string &frame_id,
    const float_t voxel_size
) {
    this->_voxel_grid_map.reset(new vgm_msgs::msg::VoxelGridMap());

    this->_voxel_grid_map->voxel_size = voxel_size;

    pcl::PointXYZL min_pt, max_pt;
    pcl::getMinMax3D(pointsmap, min_pt, max_pt);

    this->_voxel_grid_map->voxels_center.x = (min_pt.x + max_pt.x) * 0.5F;
    this->_voxel_grid_map->voxels_center.y = (min_pt.y + max_pt.y) * 0.5F;
    this->_voxel_grid_map->voxels_center.z = (min_pt.z + max_pt.z) * 0.5F;

    XYZ<long> voxels_minus = {
        static_cast<long>(std::roundf((min_pt.x - this->_voxel_grid_map->voxels_center.x) / this->_voxel_grid_map->voxel_size - 0.5F)),
        static_cast<long>(std::roundf((min_pt.y - this->_voxel_grid_map->voxels_center.y) / this->_voxel_grid_map->voxel_size - 0.5F)),
        static_cast<long>(std::roundf((min_pt.z - this->_voxel_grid_map->voxels_center.z) / this->_voxel_grid_map->voxel_size - 0.5F)),
    };
    XYZ<long> voxels_plus = {
        static_cast<long>(std::roundf((max_pt.x - this->_voxel_grid_map->voxels_center.x) / this->_voxel_grid_map->voxel_size + 0.5F)),
        static_cast<long>(std::roundf((max_pt.y - this->_voxel_grid_map->voxels_center.y) / this->_voxel_grid_map->voxel_size + 0.5F)),
        static_cast<long>(std::roundf((max_pt.z - this->_voxel_grid_map->voxels_center.z) / this->_voxel_grid_map->voxel_size + 0.5F))
    };

    this->_voxel_grid_map->voxels_origin.x = -static_cast<size_t>(voxels_minus.x);
    this->_voxel_grid_map->voxels_origin.y = -static_cast<size_t>(voxels_minus.y);
    this->_voxel_grid_map->voxels_origin.z = -static_cast<size_t>(voxels_minus.z);

    this->_voxel_grid_map->voxels_len.x = static_cast<size_t>(voxels_plus.x - voxels_minus.x);
    this->_voxel_grid_map->voxels_len.y = static_cast<size_t>(voxels_plus.y - voxels_minus.y);
    this->_voxel_grid_map->voxels_len.z = static_cast<size_t>(voxels_plus.z - voxels_minus.z);

    std::deque<std::deque<std::deque<Voxel> > > tmp_voxel_grid;
    tmp_voxel_grid.resize(this->_voxel_grid_map->voxels_len.z, std::deque<std::deque<Voxel> >(this->_voxel_grid_map->voxels_len.y, std::deque<Voxel>(this->_voxel_grid_map->voxels_len.x)));

    #ifdef _OPENMP
        #pragma omp parallel
    #endif
    {
        for (size_t z = 0UL; z < this->_voxel_grid_map->voxels_len.z; z++) {
            #ifdef _OPENMP
                #pragma omp for
            #endif
            for (size_t y = 0UL; y < this->_voxel_grid_map->voxels_len.y; y++) for (size_t x = 0UL; x < this->_voxel_grid_map->voxels_len.x; x++) {
                tmp_voxel_grid[z][y][x].min.x = static_cast<float_t>(voxels_minus.x + static_cast<long>(x)) * this->_voxel_grid_map->voxel_size + this->_voxel_grid_map->voxels_center.x;
                tmp_voxel_grid[z][y][x].min.y = static_cast<float_t>(voxels_minus.y + static_cast<long>(y)) * this->_voxel_grid_map->voxel_size + this->_voxel_grid_map->voxels_center.y;
                tmp_voxel_grid[z][y][x].min.z = static_cast<float_t>(voxels_minus.z + static_cast<long>(z)) * this->_voxel_grid_map->voxel_size + this->_voxel_grid_map->voxels_center.z;

                tmp_voxel_grid[z][y][x].max.x = static_cast<float_t>(voxels_minus.x + static_cast<long>(x) + 1L) * this->_voxel_grid_map->voxel_size + this->_voxel_grid_map->voxels_center.x;
                tmp_voxel_grid[z][y][x].max.y = static_cast<float_t>(voxels_minus.y + static_cast<long>(y) + 1L) * this->_voxel_grid_map->voxel_size + this->_voxel_grid_map->voxels_center.y;
                tmp_voxel_grid[z][y][x].max.z = static_cast<float_t>(voxels_minus.z + static_cast<long>(z) + 1L) * this->_voxel_grid_map->voxel_size + this->_voxel_grid_map->voxels_center.z;
            }
        }

        #ifdef _OPENMP
            #pragma omp for
        #endif
        for (size_t i = 0UL; i < pointsmap.size(); i++) {
            size_t idx_x = static_cast<size_t>(std::floor((pointsmap.points[i].x - this->_voxel_grid_map->voxels_center.x + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.x) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size));
            size_t idx_y = static_cast<size_t>(std::floor((pointsmap.points[i].y - this->_voxel_grid_map->voxels_center.y + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.y) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size));
            size_t idx_z = static_cast<size_t>(std::floor((pointsmap.points[i].z - this->_voxel_grid_map->voxels_center.z + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.z) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size));

            #ifdef _OPENMP
                #pragma omp critical
            #endif
            {
                tmp_voxel_grid[idx_z][idx_y][idx_x].points.push_back(pointsmap.points[i]);
            }
        }
    }

    for (size_t z = 0UL; z < this->_voxel_grid_map->voxels_len.z; z++) {
        vgm_msgs::msg::VoxelXY tmp_voxel_xy;

        for (size_t y = 0UL; y < this->_voxel_grid_map->voxels_len.y; y++) {
            vgm_msgs::msg::VoxelX tmp_voxel_x;

            for (size_t x = 0UL; x < this->_voxel_grid_map->voxels_len.x; x++) {
                vgm_msgs::msg::VoxelPoints tmp_voxelpoints;

                Voxel *tmp_voxel = &(tmp_voxel_grid.front().front().front());

                tmp_voxelpoints.min.x = tmp_voxel->min.x;
                tmp_voxelpoints.min.y = tmp_voxel->min.y;
                tmp_voxelpoints.min.z = tmp_voxel->min.z;

                tmp_voxelpoints.max.x = tmp_voxel->max.x;
                tmp_voxelpoints.max.y = tmp_voxel->max.y;
                tmp_voxelpoints.max.z = tmp_voxel->max.z;

                pcl::toROSMsg(tmp_voxel->points, tmp_voxelpoints.points);

                tmp_voxel_grid.front().front().pop_front();
                tmp_voxel_x.x.push_back(tmp_voxelpoints);
            }
            tmp_voxel_grid.front().pop_front();
            tmp_voxel_xy.y.push_back(tmp_voxel_x);
        }
        tmp_voxel_grid.pop_front();
        this->_voxel_grid_map->z.push_back(tmp_voxel_xy);
    }

    this->_voxel_grid_map->voxels_min = this->_voxel_grid_map->z[0].y[0].x[0].min;
    this->_voxel_grid_map->voxels_max = this->_voxel_grid_map->z[this->_voxel_grid_map->voxels_len.z - 1UL].y[this->_voxel_grid_map->voxels_len.y - 1UL].x[this->_voxel_grid_map->voxels_len.x - 1UL].max;
    this->_voxel_grid_map->header.frame_id = frame_id;

    RCLCPP_INFO_STREAM((*this->_logger), "Load " << pointsmap.size() << " points.");
}

template <class VoxelGridMapPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::_create_frustum_points(
    const Range &depth_range,
    const LRTB<float_t> &tan,
    pcl::PointCloud<pcl::PointXYZL> &dst
) {
    dst.points.resize(9);

    //  原点
    pcl::PointXYZL origin;
    origin.x = 0.0f; origin.y = 0.0f; origin.z = 0.0f;
    dst.points[frustumPointIndex::Origin] = origin;

    float_t depth_front, depth_back;

    if (std::isinf(depth_range.max) == true) depth_back = depth_range.min + 1.0f;
    else depth_back = depth_range.max;

    if (depth_range.min > 0.0f) depth_front = depth_range.min;
    else depth_front = depth_back * 0.5f;

    //  前面左下
    pcl::PointXYZL front_bottom_left;
    front_bottom_left.x = tan.l * depth_front;
    front_bottom_left.y = tan.b * depth_front;
    front_bottom_left.z = depth_front;
    dst.points[frustumPointIndex::FrontBottomLeft] = front_bottom_left;

    //  前面左上
    pcl::PointXYZL front_top_left;
    front_top_left.x = tan.l * depth_front;
    front_top_left.y = tan.t * depth_front;
    front_top_left.z = depth_front;
    dst.points[frustumPointIndex::FrontTopLeft] = front_top_left;

    //  前面右上
    pcl::PointXYZL front_top_right;
    front_top_right.x = tan.r * depth_front;
    front_top_right.y = tan.t * depth_front;
    front_top_right.z = depth_front;
    dst.points[frustumPointIndex::FrontTopRight] = front_top_right;

    //  前面右下
    pcl::PointXYZL front_bottom_right;
    front_bottom_right.x = tan.r * depth_front;
    front_bottom_right.y = tan.b * depth_front;
    front_bottom_right.z = depth_front;
    dst.points[frustumPointIndex::FrontBottomRight] = front_bottom_right;

    //  後面左下
    pcl::PointXYZL back_bottom_left;
    back_bottom_left.x = tan.l * depth_back;
    back_bottom_left.y = tan.b * depth_back;
    back_bottom_left.z = depth_back;
    dst.points[frustumPointIndex::BackBottomLeft] = back_bottom_left;

    //  後面左上
    pcl::PointXYZL back_top_left;
    back_top_left.x = tan.l * depth_back;
    back_top_left.y = tan.t * depth_back;
    back_top_left.z = depth_back;
    dst.points[frustumPointIndex::BackTopLeft] = back_top_left;

    //  後面右上
    pcl::PointXYZL back_top_right;
    back_top_right.x = tan.r * depth_back;
    back_top_right.y = tan.t * depth_back;
    back_top_right.z = depth_back;
    dst.points[frustumPointIndex::BackTopRight] = back_top_right;

    //  後面右下
    pcl::PointXYZL back_bottom_right;
    back_bottom_right.x = tan.r * depth_back;
    back_bottom_right.y = tan.b * depth_back;
    back_bottom_right.z = depth_back;
    dst.points[frustumPointIndex::BackBottomRight] = back_bottom_right;
}

template <class VoxelGridMapPtrT>
bool VoxelGridMap<VoxelGridMapPtrT>::_voxel_frontside(
    const vgm_msgs::msg::VoxelPoints &voxel,
    const Eigen::Vector3f &normal,
    const Eigen::Vector3f &point_on_plane
) {
    //  平面から最も遠い点を導出
    Eigen::Vector3f voxel_apex_max(
        (normal[Axis::X] > 0.0f)? voxel.max.x : voxel.min.x,
        (normal[Axis::Y] > 0.0f)? voxel.max.y : voxel.min.y,
        (normal[Axis::Z] > 0.0f)? voxel.max.z : voxel.min.z
    );
    Eigen::Vector3f voxel_apex_min(
        (normal[Axis::X] < 0.0f)? voxel.max.x : voxel.min.x,
        (normal[Axis::Y] < 0.0f)? voxel.max.y : voxel.min.y,
        (normal[Axis::Z] < 0.0f)? voxel.max.z : voxel.min.z
    );

    //  0以下の場合，視錐台に含まれる
    return (point_on_plane - voxel_apex_max).dot(normal) <= 0.0f || (point_on_plane - voxel_apex_min).dot(normal) <= 0.0f;
}

template <class VoxelGridMapPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::_voxels_include_frustum(
    const Eigen::Vector3f &translation,
    const Eigen::Quaternionf &quaternion,
    const Range &depth_range,
    const LRTB<float_t> &tan,
    std::vector<XYZ<size_t> > &dst_voxel_indexs
) {
    pcl::PointCloud<pcl::PointXYZL> frustum_points;
    this->_create_frustum_points(depth_range, tan, frustum_points);

    Eigen::Vector3f i_translation;
    Eigen::Quaternionf i_quaternion;
    this->_invert_transform(translation, quaternion, i_translation, i_quaternion);
    this->_transform_pointcloud(frustum_points, frustum_points, i_translation, i_quaternion);

    this->_voxels_include_frustum(frustum_points, depth_range, dst_voxel_indexs);
}

template <class VoxelGridMapPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::_voxels_include_frustum(
    const pcl::PointCloud<pcl::PointXYZL> &frustum_points,
    const Range &depth_range,
    std::vector<XYZ<size_t> > &dst_voxel_indexs
) {
    /*  0: FrontBottomLeft
        1: FrontTopLeft
        2: FrontTopRight
        3: FrontBottomRight
        4: BackBottomLeft
        5: BackTopLeft
        6: BackTopRight
        7: BackBottomRight
        8: Origin           */
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > points(9);
    for (size_t i = 0ul; i < 9ul; i++) {
        points[i][Axis::X] = frustum_points.points[i].x;
        points[i][Axis::Y] = frustum_points.points[i].y;
        points[i][Axis::Z] = frustum_points.points[i].z;
    }

    /*  0: Origin -> BackBottomLeft
        1: Origin -> BackTopLeft
        2: Origin -> BackTopRight
        3: Origin -> BackBottomRight
        4: FrontBottomRight -> FrontBottomLeft
        5: FrontBottomRight -> FrontTopRight
        6: BackTopLeft -> BackBottomLeft
        7: BackTopLeft -> BackTopRight          */
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vectors(8);
    for (size_t i = 0ul; i < 4ul; i++)
        vectors[i] = points[i + 4ul] - points[frustumPointIndex::Origin];
    for (size_t i = 0ul; i < 2ul; i++) {
        vectors[i + 4ul] = points[i * 2ul] - points[frustumPointIndex::FrontBottomRight];
        vectors[i + 6ul] = points[i * 2ul + 4ul] - points[frustumPointIndex::BackTopLeft];
    }

    /*  0: Left
        1: Top
        2: Right
        3: Bottom
        4: Front
        5: Back     */
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > normals(6);
    for (size_t i = 0ul; i < 4ul; i++) {
        size_t i_next = (i >= 3ul)? 0ul : i + 1ul;
        normals[i] = vectors[i].cross(vectors[i_next]);
    }
    for (size_t i = 0ul; i < 2ul; i++)
        normals[i + 4ul] = vectors[i * 2ul + 4ul].cross(vectors[i * 2ul + 5ul]);

    /*  0: FrontBottomLeft
        1: FrontTopLeft
        2: FrontTopRight
        3: FrontBottomRight
        4: BackBottomLeft
        5: BackTopLeft
        6: BackTopRight
        7: BackBottomRight  */
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vertexes(8);

    if (depth_range.min > 0.0f) for (size_t i = 0ul; i < 4ul; i++) vertexes[i] = points[i];
    else for (size_t i = 0ul; i < 4ul; i++) vertexes[i] = points[frustumPointIndex::Origin];

    if (std::isinf(depth_range.max) == true)
        for (size_t i = 0ul; i < 4ul; i++) {
            Eigen::Vector3f voxels_lim(
                (vectors[i][Axis::X] < 0.0f)? this->_voxel_grid_map->voxels_min.x : this->_voxel_grid_map->voxels_max.x,
                (vectors[i][Axis::Y] < 0.0f)? this->_voxel_grid_map->voxels_min.y : this->_voxel_grid_map->voxels_max.y,
                (vectors[i][Axis::Z] < 0.0f)? this->_voxel_grid_map->voxels_min.z : this->_voxel_grid_map->voxels_max.z
            );
            float_t t = std::min({
                (voxels_lim[Axis::X] - points[frustumPointIndex::Origin][Axis::X]) / vectors[i][Axis::X],
                (voxels_lim[Axis::Y] - points[frustumPointIndex::Origin][Axis::Y]) / vectors[i][Axis::Y],
                (voxels_lim[Axis::Z] - points[frustumPointIndex::Origin][Axis::Z]) / vectors[i][Axis::Z]
            });
            vertexes[i + 4ul] = points[frustumPointIndex::Origin] + vectors[i] * t;
        }
    else
        for (size_t i = 4ul; i < 8ul; i++)
            vertexes[i] = points[i];

    Eigen::Vector3f point_min = vertexes[0];
    Eigen::Vector3f point_max = vertexes[0];
    for (size_t i = 1ul; i < 8ul; i++) {
        if (vertexes[i][Axis::X] < point_min[Axis::X]) point_min[Axis::X] = vertexes[i][Axis::X];
        else if (point_max[Axis::X] < vertexes[i][Axis::X]) point_max[Axis::X] = vertexes[i][Axis::X];

        if (vertexes[i][Axis::Y] < point_min[Axis::Y]) point_min[Axis::Y] = vertexes[i][Axis::Y];
        else if (point_max[Axis::Y] < vertexes[i][Axis::Y]) point_max[Axis::Y] = vertexes[i][Axis::Y];

        if (vertexes[i][Axis::Z] < point_min[Axis::Z]) point_min[Axis::Z] = vertexes[i][Axis::Z];
        else if (point_max[Axis::Z] < vertexes[i][Axis::Z]) point_max[Axis::Z] = vertexes[i][Axis::Z];
    }

    XYZ<float_t> v_idx_min_f = {
        std::floor((point_min[Axis::X] - this->_voxel_grid_map->voxels_center.x + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.x) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size),
        std::floor((point_min[Axis::Y] - this->_voxel_grid_map->voxels_center.y + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.y) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size),
        std::floor((point_min[Axis::Z] - this->_voxel_grid_map->voxels_center.z + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.z) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size)
    };
    XYZ<float_t> v_idx_max_f = {
        std::ceil((point_max[Axis::X] - this->_voxel_grid_map->voxels_center.x + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.x) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size),
        std::ceil((point_max[Axis::Y] - this->_voxel_grid_map->voxels_center.y + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.y) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size),
        std::ceil((point_max[Axis::Z] - this->_voxel_grid_map->voxels_center.z + static_cast<float_t>(this->_voxel_grid_map->voxels_origin.z) * this->_voxel_grid_map->voxel_size) / this->_voxel_grid_map->voxel_size)
    };

    XYZ<size_t> v_idx_min = {
        (v_idx_min_f.x < 0.0f)? 0ul : static_cast<size_t>(v_idx_min_f.x),
        (v_idx_min_f.y < 0.0f)? 0ul : static_cast<size_t>(v_idx_min_f.y),
        (v_idx_min_f.z < 0.0f)? 0ul : static_cast<size_t>(v_idx_min_f.z)
    };
    XYZ<size_t> v_idx_max = {
        (v_idx_max_f.x > static_cast<float_t>(this->_voxel_grid_map->voxels_len.x))? this->_voxel_grid_map->voxels_len.x : (v_idx_max_f.x < 0.0f)? 0ul : static_cast<size_t>(v_idx_max_f.x),
        (v_idx_max_f.y > static_cast<float_t>(this->_voxel_grid_map->voxels_len.y))? this->_voxel_grid_map->voxels_len.y : (v_idx_max_f.y < 0.0f)? 0ul : static_cast<size_t>(v_idx_max_f.y),
        (v_idx_max_f.z > static_cast<float_t>(this->_voxel_grid_map->voxels_len.z))? this->_voxel_grid_map->voxels_len.z : (v_idx_max_f.z < 0.0f)? 0ul : static_cast<size_t>(v_idx_max_f.z)
    };

    dst_voxel_indexs.clear();

    for (size_t z = v_idx_min.z; z < v_idx_max.z; z++) for (size_t y = v_idx_min.y; y < v_idx_max.y; y++) for (size_t x = v_idx_min.x; x < v_idx_max.x; x++) {
        bool in_frustum = true;

        for (size_t i = 0ul; i < 4ul; i++)
            in_frustum &= this->_voxel_frontside(this->_voxel_grid_map->z[z].y[y].x[x], normals[i], points[frustumPointIndex::Origin]);

        if (depth_range.min > 0.0f)
            in_frustum &= this->_voxel_frontside(this->_voxel_grid_map->z[z].y[y].x[x], normals[frustumSurfaceIndex::Front], points[frustumPointIndex::FrontBottomRight]);

        if (std::isinf(depth_range.max) == false)
            in_frustum &= this->_voxel_frontside(this->_voxel_grid_map->z[z].y[y].x[x], normals[frustumSurfaceIndex::Back], points[frustumPointIndex::BackTopLeft]);

        if (in_frustum == false) continue;

        XYZ<size_t> voxel_index = {x, y, z};
        dst_voxel_indexs.push_back(voxel_index);
    }
}

template <class VoxelGridMapPtrT>
void VoxelGridMap<VoxelGridMapPtrT>::_create_depthmap(
    const std::vector<XYZ<size_t> > &voxel_indexs,
    const Eigen::Vector3f &translation,
    const Eigen::Quaternionf &quaternion,
    const CameraParam &camera_param,
    const XY<float_t> &shape_f,
    const LRTB<float_t> &tan,
    const Range &depth_range,
    cv::Mat &depth
) {
    depth = cv::Mat(camera_param.height, camera_param.width, CV_32FC1, cv::Scalar_<float_t>(INFINITY));

    size_t voxels_len = voxel_indexs.size();

    std::vector<cv::Mat> processing_depth;
    #ifdef _OPENMP
        int threads = omp_get_max_threads();
        processing_depth.assign(threads, depth.clone());
        #pragma omp parallel
    #else
        int threads = 1;
        processing_depth.assign(threads, depth.clone());
    #endif
    {
        #ifdef _OPENMP
            #pragma omp for
        #endif
        for (size_t i = 0ul; i < voxels_len; i++) {
            pcl::PointCloud<pcl::PointXYZL> points;
            pcl::fromROSMsg(this->_voxel_grid_map->z[voxel_indexs[i].z].y[voxel_indexs[i].y].x[voxel_indexs[i].x].points, points);
            this->_transform_pointcloud(points, points, translation, quaternion);
            this->_depth_filter(points, points, depth_range);

            size_t p_len = points.points.size();

            for (size_t p = 0; p < p_len; p++) {
                pcl::PointXYZL *src_point = &(points.points[p]);

                float_t th_L = src_point->z * tan.l;
                float_t th_R = src_point->z * tan.r;
                float_t th_T = src_point->z * tan.t;
                float_t th_B = src_point->z * tan.b;

                if (th_L <= src_point->x && src_point->x <= th_R && th_T <= src_point->y && src_point->y <= th_B) {
                    int x, y;

                    if (src_point->x < 0.0f) x = static_cast<int>(camera_param.Cx - roundf(camera_param.Cx * src_point->x / (src_point->z * tan.l)));
                    else x = static_cast<int>(camera_param.Cx + roundf((shape_f.x - camera_param.Cx) * src_point->x / (src_point->z * tan.r)));

                    if (src_point->y < 0.0f) y = static_cast<int>(camera_param.Cy - roundf(camera_param.Cy * src_point->y / (src_point->z * tan.t)));
                    else y = static_cast<int>(camera_param.Cy + roundf((shape_f.y - camera_param.Cy) * src_point->y / (src_point->z * tan.b)));

                    if (0 <= x && x < camera_param.width && 0 <= y && y < camera_param.height) {
                        #ifdef _OPENMP
                            float_t *cv_dst_ptr = processing_depth[omp_get_thread_num()].ptr<float_t>(y);
                        #else
                            float_t *cv_dst_ptr = processing_depth[0].ptr<float_t>(y);
                        #endif
                        if (cv_dst_ptr[x] > src_point->z) {
                            cv_dst_ptr[x] = src_point->z;
                        }
                    }
                }
            }
        }

        #ifdef _OPENMP
            #pragma omp for
        #endif
        for (int y = 0; y < depth.rows; y++) {
            std::vector<float_t*> depth_ptrs;
            for (int i = 0; i < threads; i++) {
                depth_ptrs.push_back(processing_depth[i].ptr<float_t>(y));
            }
            float_t* cv_depth_ptr = depth.ptr<float_t>(y);
            for (int x = 0; x < depth.cols; x++) {
                for (int i = 0; i < threads; i++) {
                    cv_depth_ptr[x] = std::min(cv_depth_ptr[x], depth_ptrs[i][x]);
                }
            }
        }
    }
}

template <>
void VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>::set_pointsmap(
    const std::vector<std::string> &paths,
    const std::string &frame_id,
    const float_t voxel_size
) {
    size_t paths_len = paths.size();
    pcl::PointCloud<pcl::PointXYZL> pointsmap, part;

    for (size_t i = 0UL; i < paths_len; i++) {
        if (pcl::io::loadPCDFile(paths[i], part) == -1) {
            RCLCPP_ERROR_STREAM((*this->_logger), "Load failed \"" << paths[i] << "\"");
        }
        else {
            RCLCPP_INFO_STREAM((*this->_logger), "Load \"" << paths[i] << "\" (" << part.size() << " points)");
            pointsmap += part;
        }
    }

    this->_set_voxel_grid(pointsmap, frame_id, voxel_size);
}

template <>
void VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>::set_pointsmap(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointsmap,
    const float_t voxel_size
) {
    pcl::PointCloud<pcl::PointXYZL> tmp_points;
    pcl::fromROSMsg(*pointsmap, tmp_points);
    this->_set_voxel_grid(tmp_points, pointsmap->header.frame_id, voxel_size);
}

template <>
VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>::VoxelGridMap(
    const rclcpp::Logger &logger,
    const std::string &path,
    const std::string &frame_id,
    const float_t voxel_size
) {
    this->set_pointsmap({path}, frame_id, voxel_size);
    this->_logger = std::make_shared<rclcpp::Logger>(logger);
}

template <>
VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>::VoxelGridMap(
    const rclcpp::Logger &logger,
    const std::vector<std::string> &paths,
    const std::string &frame_id,
    const float_t voxel_size
) {
    this->set_pointsmap(paths, frame_id, voxel_size);
    this->_logger = std::make_shared<rclcpp::Logger>(logger);
}

template <>
VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>::VoxelGridMap(
    const rclcpp::Logger &logger,
    const sensor_msgs::msg::PointCloud2::SharedPtr pointsmap,
    const float_t voxel_size
) {
    this->set_pointsmap(pointsmap, voxel_size);
    this->_logger = std::make_shared<rclcpp::Logger>(logger);
}

template class VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>;

template void VoxelGridMap<vgm_msgs::msg::VoxelGridMap::SharedPtr>::create_depthmap(
    const sensor_msgs::msg::CameraInfo::SharedPtr &camerainfo,
    const geometry_msgs::msg::TransformStamped &tf,
    cv::Mat &depthmap,
    const Range &depth_range
);
