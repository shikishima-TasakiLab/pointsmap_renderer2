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
