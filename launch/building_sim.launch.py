"""Mở căn nhà trong Gazebo Fortress cùng TurtleBot 4.

Không tự sửa URDF, cảm biến, TF, joint state hoặc đường truyền cmd_vel.
Dùng bản đồ SLAM đã lưu. Robot khởi đầu trên dock; Undock trước khi lái.
"""
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def include(package, filename, **arguments):
    path = os.path.join(share(package), 'launch', filename)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=arguments.items())


def generate_launch_description():
    project = share('tb4_voice_navigation')
    world = LaunchConfiguration('world')
    world_file = PathJoinSubstitution([project, 'worlds', [world, '.sdf']])
    model = LaunchConfiguration('model')
    headless = LaunchConfiguration('headless')
    resource_paths = [os.path.dirname(share(package)) for package in
                      ['turtlebot4_description', 'irobot_create_description']]
    # model:// trong world và DAE cần tìm được thư mục model cục bộ.
    resource_paths.append(os.path.join(project, 'models'))
    resource_paths.append(os.path.join(project, 'worlds'))
    resource_paths.append(os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))
    # Cùng plugin GUI với launch TurtleBot chuẩn (HMI và teleop).
    gui_paths = [os.path.join(share(package), 'lib') for package in
                 ['turtlebot4_ignition_gui_plugins', 'irobot_create_ignition_plugins']]
    gui_paths.append(os.environ.get('IGN_GUI_PLUGIN_PATH', ''))
    gui_config = PathJoinSubstitution([share('turtlebot4_ignition_bringup'),
                                       'gui', model, 'gui.config'])
    # Truyền thẳng môi trường vào tiến trình Gazebo, tránh launch lồng nhau
    # dựng lại environment và làm mất thư mục model của căn nhà.
    gazebo_env = {
        'IGN_GAZEBO_RESOURCE_PATH': os.pathsep.join(resource_paths),
        'IGN_GUI_PLUGIN_PATH': os.pathsep.join(gui_paths),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
        ]),
    }
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        # Tên này dùng cho bridge cảm biến, khác với đường dẫn file world.
        DeclareLaunchArgument('world', default_value='small_house', choices=['small_house']),
        # Khởi động đúng Fortress 6 và giữ môi trường model/plugin đã chuẩn bị.
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '-r', '-v', '2',
                 '--gui-config', gui_config, '--force-version', '6'],
            additional_env=gazebo_env, output='screen',
            condition=UnlessCondition(headless), on_exit=Shutdown()),
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '-s', '-r', '-v', '2',
                 '--force-version', '6'],
            additional_env=gazebo_env, output='screen',
            condition=IfCondition(headless), on_exit=Shutdown()),
        # Cả chế độ có cửa sổ và headless đều cần đồng hồ mô phỏng.
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        # Khởi tạo robot, TF và controller bằng launch chuẩn TurtleBot 4.
        # Giữ tham số spawn ở phạm vi ngoài vì callback khởi động controller
        # còn dùng namespace sau khi nhóm spawn đã chạy xong.
        include('turtlebot4_ignition_bringup', 'turtlebot4_spawn.launch.py',
                world=world, model=model, namespace='',
                use_sim_time='true', rviz='false', slam='false',
                localization='false', nav2='false', x='0.0', y='0.0', z='0.0', yaw='0.0'),
    ])
