"""Chạy phần giọng nói + gửi goal. Khởi chạy mô phỏng/Nav2 ở terminal riêng."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('tb4_voice_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('regions_file', default_value=share+'/maps/regions.yaml'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_listening', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_speech', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('use_hotwords', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('microphone_index', default_value='-1'),
        DeclareLaunchArgument('command_max_age', default_value='10.0'),
        DeclareLaunchArgument('audio_gain', default_value='1.8'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='tb4_voice_navigation', executable='voice_navigation_node', output='screen',
             parameters=[{
                 'regions_file': LaunchConfiguration('regions_file'),
                 'command_max_age': ParameterValue(LaunchConfiguration('command_max_age'), value_type=float),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
        Node(package='tb4_voice_navigation', executable='speech_node', output='screen',
             condition=IfCondition(LaunchConfiguration('enable_speech')),
             parameters=[{
                 'model_path': LaunchConfiguration('model_path'),
                 'use_hotwords': ParameterValue(LaunchConfiguration('use_hotwords'), value_type=bool),
                 'regions_file': LaunchConfiguration('regions_file'),
                 'command_max_age': ParameterValue(LaunchConfiguration('command_max_age'), value_type=float),
                 'start_listening': ParameterValue(LaunchConfiguration('start_listening'), value_type=bool),
                 'microphone_index': ParameterValue(LaunchConfiguration('microphone_index'), value_type=int),
                 'audio_gain': ParameterValue(LaunchConfiguration('audio_gain'), value_type=float),
             }]),
        # Đóng cửa sổ sẽ kết thúc bộ node giọng nói; node điều hướng yêu cầu hủy goal.
        # Mô phỏng/Nav2 chạy ở terminal riêng nên không bị đóng theo cửa sổ này.
        Node(package='tb4_voice_navigation', executable='voice_control_gui', output='screen',
             condition=IfCondition(LaunchConfiguration('gui')), on_exit=Shutdown(),
             parameters=[{
                 'regions_file': LaunchConfiguration('regions_file'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
