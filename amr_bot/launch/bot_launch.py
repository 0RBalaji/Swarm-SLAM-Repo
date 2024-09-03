import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument

from launch.actions import ExecuteProcess

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    #package find
    pkg_path = os.path.join(get_package_share_directory('amr_bot'))
    bot_file = os.path.join(pkg_path, 'design', 'amr.urdf.xacro')
    #bot_design_config = xacro.process_file(bot_file)
    bot_design_config = Command(['xacro ', bot_file,  ' sim_mode:=', use_sim_time])
    
    #Robot state publisher node creation
    para = {'robot_description': bot_design_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters= [para]
    )
    
    #Joint state publisher node creation
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
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
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )
    return LaunchDescription([
        # Nodes
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        # diff_drive_spawner,
        # joint_broad_spawner,
        node_rviz2,
    ])
