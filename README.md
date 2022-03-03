# Pointsmap Renderer

## Docker Image

- pull
    ```bash
    docker pull shikishimatasakilab/pointsmap_renderer:foxy
    ```
- build
    ```bash
    ./docker/build.foxy.sh
    ```

## Start a Docker Container

1. Start a Docker container with the following command.
    ```bash
    ./docker/run.sh
    ```

1. Build source code.
    ```bash
    colcon build
    source /workspace/install/setup.bash
    ```

## How to use

### ROS2 Launch

Write the following configuration to the YAML file.

Ex) pmod.py

|Key|Value|Default|
|----|--|------|
|`voxel_size`|The length of a side of the voxel that stores the point cloud maps. [m]|`10.0`|
|`init_maps`|List of point cloud map paths to be loaded at startup.|None|
|`map_frameid`|Coordinate system of the point cloud maps to be loaded at startup.|`map`|

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='pmod',
            package='pointsmap_renderer',
            executable='vgm_pub',
            parameters=[
                {'voxel_size': 0.2},
            ],
        ),
        Node(
            namespace='pmod',
            package='pointsmap_renderer',
            executable='pointsmap_renderer',
            parameters=[
                {'hz': 5.0},
                {'queue_size': 2},
                {'depth_min': 1.0},
                {'depth_max': 10.0},
            ],
            remappings=[
                ('camera_info', '/pmod/sparse_depth/camera_info'),
                ('sparse_depth', '/pmod/sparse_depth'),
            ]
        ),
    ])
```

## Overview of each node

### pointsmap_renderer/vgm_pub

Convert the PCD files specified in `init_maps` or the maps in `/points_map` topic to a Voxel Grid Map and publish it.

- Publications:
  - `/voxel_grid_map` [pointsmap_renderer/VoxelGridMap]

- Subscriptions:
  - `/points_map` [sensor_msgs/PointCloud2]

- Params:
  - `voxel_size`: The length of a side of the voxel that stores the point cloud maps. [m] (`10.0`)
  - `init_maps`: List of point cloud map paths to be loaded at startup. (None)
  - `map_frameid`: Coordinate system of the point cloud maps to be loaded at startup. (`map`)

### pointsmap_renderer/pointsmap_renderer_node

Generate depth maps projected from point cloud maps from `~/camera_info`, `/voxel_grid_map` and TF, and publish them as `~/sparse_depth`.

- Publications:
  - `~/sparse_depth` [sensor_msgs/Image]

- Subscriptions:
  - `~/camera_info` [sensor_msgs/Image]
  - `/tf` [tf2_msgs/TFMessage]
  - `/tf_static` [tf2_msgs/TFMessage]
  - `/voxel_grid_map` [pointsmap_renderer/VoxelGridMap]

- Params:
  - `hz`: Frequency of publish. [Hz] (`5.0`)
  - `depth_min`: Minimum value for depth map [m] (`0.0`)
  - `depth_max`: Maximum value for depth map [m] (`INFINITY`)
  - `queue_size`: Queue size of `~/sparse_depth` (`2`)
