import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml, ReplaceString
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    pkg_name = "twowheelbot"

    delay = 0.0

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # params_file = './src/twowheelbot/config/mapper_params_online_async.yaml'
    # map = './src/twowheelbot/ros2_map_map.yaml'

    sim_path = os.path.join(get_package_share_directory(pkg_name),
                            'launch', 'bot_sim.launch.py')
    
    slam_path = os.path.join(get_package_share_directory(pkg_name),
                            'launch', 'online_async_launch.py')
    
    localization_path = os.path.join(get_package_share_directory(pkg_name),
                            'launch', 'localization_launch.py')
    
    navigation_path = os.path.join(get_package_share_directory(pkg_name),
                            'launch', 'navigation_launch.py')
    
    params_file = os.path.join(get_package_share_directory(pkg_name),
                              'config', 'mapper_params_online_async.yaml')
    
    map_file = os.path.join(get_package_share_directory(pkg_name), 'ros2_map_map.yaml')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file}
    
    # configured_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=params_file,
    #         root_key=namespace,
    #         param_rewrites=param_substitutions,
    #         convert_types=True),
    #     allow_substs=True)
    
    sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_path),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'namespace':namespace,
                            'x_pose':x_pose, 'y_pose':y_pose}.items())

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_path),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'map':map_file,
                            'namespace':namespace,
                            'params_file':params_file}.items())
    
    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_path),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'namespace':namespace,
                            'map':'./src/twowheelbot/ros2_map_map.yaml'}.items()
    )

    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_path),
        launch_arguments = {'use_sim_time': use_sim_time, 'namespace':namespace}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument('namespace', default_value='', description='pass the namespace with the _ at end'),
        DeclareLaunchArgument('x_pose', default_value='2.5', description='X position for the robot spawn'),
        DeclareLaunchArgument('y_pose', default_value='4', description='Y position for the robot spawn'),
        # sim_node,

        TimerAction(
            period= delay + 1.0,
            actions=[sim_node]
        ),
        
        TimerAction(
            period=delay + 5.0,
            actions=[slam_node]
        ),
        # TimerAction(
        #     period=delay + 5.0,  # Delay by 5 seconds from start (3 seconds after SLAM)
        #     actions=[localization_node]
        # ),
        # TimerAction(
        #     period=delay + 5.0,
        #     actions=[navigation_node]
        # )
    ])