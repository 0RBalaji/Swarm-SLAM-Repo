import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "twowheelbot"
    pkg_path = os.path.join(get_package_share_directory(pkg_name), 'launch', 'bot_launch.py')
    gazebo_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name),'config','gazebo_params.yaml')
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_path]),
        launch_arguments = {'use_sim_time': 'true'}.items()
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_path]), launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )
    
    spawn = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-topic','robot_description', '-entity', 'amr_bot'],
        output = 'screen'
    )

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )
    
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diff_cont"],
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_broad"],
    # )
    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn
    ])