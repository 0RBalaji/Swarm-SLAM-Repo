import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
# from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    pkg_path = get_package_share_directory('slam_nav')
    launch_dir = os.path.join(pkg_path, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                #   ('/scan','scan'),
                  ('/map','merged_map'),
                #   ('/map_metadata','map_metadata'),
                #   ('/goal_pose','goal_pose'),
                #   ('/global_costmap','global_costmap'),
                #   ('/local_costmap','local_costmap'),
                #   ('/cmd_vel','cmd_vel'),
                #   ('/smoother_server','smoother_server'),
                #   ('/odom','odom'),
                #   ('/controller_server','controller_server'),
                #   ('/planner_server','planner_server'),
                #   ('/behavior_server','behavior_server'),
                #   ('/bt_navigator','bt_navigator'),
                #   ('/waypoint_follower','waypoint_follower'),
                #   ('/velocity_smoother','velocity_smoother')
                  ]

    
    # Create temporary YAML files with substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    
    # params_file = ReplaceString(
    #     source_file=params_file,
    #     replacements={'<robot_namespace>': ('/', namespace)},
    #     condition=IfCondition(use_namespace))

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to the map YAML file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file for all launched nodes')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level')
    
    # Specified Action
    bringup_cmd_group = GroupAction([
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            namespace=namespace,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'online_async_launch.py')),
            # condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': params_file}.items()),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        #     # condition=IfCondition(PythonExpression(['not ', slam])),
        #     launch_arguments={'namespace': namespace,
        #                       'map': map_yaml_file,
        #                       'use_sim_time': use_sim_time,
        #                       'params_file': params_file,
        #                       'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': params_file,
                              'container_name': 'nav2_container'}.items()),
    ])

    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     namespace=namespace,
    #     parameters=[configured_params, {'yaml_filename': map_yaml_file}],
    #     remappings=remappings)

    # Launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add actions to launch all navigation nodes
    # ld.add_action(map_server_node)
    
    ld.add_action(bringup_cmd_group)

    return ld
