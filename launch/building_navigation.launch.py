"""Mở mô phỏng, định vị AMCL, Nav2 và RViz với bản đồ đã lưu."""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, **arguments):
    path = os.path.join(share(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=arguments.items())


def generate_launch_description():
    project = share('tb4_voice_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        include('tb4_voice_navigation', 'building_sim.launch.py',
                headless=LaunchConfiguration('headless')),
        include('turtlebot4_navigation', 'localization.launch.py',
                map=project+'/maps/map.yaml', params=project+'/config/localization.yaml',
                use_sim_time='true'),
        include('turtlebot4_navigation', 'nav2.launch.py',
                params_file=project+'/config/nav2.yaml', use_sim_time='true'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', project+'/rviz/building.rviz'],
             parameters=[{'use_sim_time': True}]),
    ])
