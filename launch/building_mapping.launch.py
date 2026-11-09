"""Quét bản đồ căn nhà; điều hướng với map đã lưu không cần launch này."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tb4_voice_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        # Lưu lựa chọn trước khi launch spawn đặt rviz=false cho RViz mặc định.
        SetLaunchConfiguration('mapping_rviz_enabled', LaunchConfiguration('rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch/building_sim.launch.py')),
            launch_arguments={'headless': LaunchConfiguration('headless')}.items()),
        # SLAM dùng scan và TF để dựng bản đồ trong lúc robot di chuyển.
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[os.path.join(share, 'config/slam.yaml'), {'use_sim_time': True}],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/scan', 'scan'),
                         ('/map', 'map'), ('/map_metadata', 'map_metadata')]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('mapping_rviz_enabled')),
             arguments=['-d', os.path.join(share, 'rviz/mapping.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
