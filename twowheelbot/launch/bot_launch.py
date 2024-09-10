import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument

from launch.actions import ExecuteProcess

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro
import launch

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    #package find
    pkg_path = os.path.join(get_package_share_directory('twowheelbot'))
    bot_file = os.path.join(pkg_path, 'designs', 'amr.urdf.xacro')
    #bot_design_config = xacro.process_file(bot_file)
    bot_design_config = Command(['xacro ', bot_file, ' sim_mode:=', use_sim_time])
    
    #Robot state publisher node creation
    para = {'robot_description': bot_design_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters= [para]
    )

    #Joint state publisher node creation
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments = [bot_file]
        parameters=[{'robot_description': Command(['xacro ', bot_file])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    #Joint state publisher gui node creation
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    
    #RVIZ launch node
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'config.rviz')
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_path, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        # Nodes
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),

        node_robot_state_publisher,
        node_joint_state_publisher,
        robot_localization_node,
        node_joint_state_publisher_gui
        node_rviz2
    ])
