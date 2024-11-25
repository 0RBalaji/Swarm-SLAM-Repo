import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    
    #package find
    pkg_path = os.path.join(get_package_share_directory('twowheelbot'))
    bot_file = os.path.join(pkg_path, 'designs', 'amr.urdf.xacro')
    
    bot_design_config = Command(['xacro ', bot_file, ' sim_mode:=', use_sim_time, ' namespace:=',namespace])

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    #Robot state publisher node creation
    para = {'robot_description': bot_design_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name='rsp',
        namespace=namespace,
        output = 'screen',
        parameters= [para],
        # remappings=[('/base_footprint','base_footprint'),
        #             ('/left_wheel','left_wheel'),
        #             ('/right_wheel','right_wheel'),
        #             ('/lidar_link','lidar_link')]
        remappings=remappings
    )

    #Joint state publisher node creation
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='jsp',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        # remappings=[('/base_footprint','base_footprint'),
        #             ('/left_wheel','left_wheel'),
        #             ('/right_wheel','right_wheel'),
        #             ('/lidar_link','lidar_link')]
        remappings=remappings
    )
    
    #RVIZ launch node
    rviz_config_file = os.path.join(pkg_path, 'config', 'viz.rviz')
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['namespace', namespace, 
                   '-d', rviz_config_file]
    )
    
    return LaunchDescription([
        # Nodes
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument('namespace', default_value='', description='pass the namespace with the _ at end'),

        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz2
    ])
