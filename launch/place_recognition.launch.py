from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('btc_desc')
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    btc_place_recognition_node = Node(
        package='btc_desc',
        executable='btc_place_recognition',
        name='btc_place_recognition',
        output='screen',
        parameters=[{
            'cloud_overlap_thr': 0.5,
            'read_bin': True,
            'setting_path': os.path.join(pkg_dir, 'config', 'config_outdoor.yaml'),
            'pcds_dir': '/mnt/h/kitti/sequences/00/velodyne',
            'pose_file': '/mnt/h/kitti/correct_pose/kitti00.txt'
        }]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'rviz_cfg', 'loop.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        rviz_arg,
        btc_place_recognition_node,
        rviz_node
    ])


