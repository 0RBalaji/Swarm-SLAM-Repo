import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    default_params_file = os.path.join(get_package_share_directory('slam_nav'),'config', 'mapper_params_online_async.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # has_node_params = HasNodeParams(source_file=params_file,
    #                                 node_name='slam_toolbox')

    # actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
    #                                        ' else "', default_params_file, '"'])

    # log_param_change = LogInfo(msg=['provided params_file ',  params_file,
    #                                 ' does not contain slam_toolbox parameters. Using default: ',
    #                                 default_params_file],
    #                            condition=UnlessCondition(has_node_params))

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace=namespace,
        name='slam_toolbox',
        output='screen',
        parameters=[
            default_params_file,
            {'use_sim_time': use_sim_time},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_footprint'},
            {'scan_topic': 'scan'},
            # {'map_yaml_file':'/home/balaji/multiamr/workshop_twowheelbot_serail'},
        ],
        remappings=[
                    ('/scan','scan'),
                    ('/map','map'),
                    ('/map_metadata','map_metadata'),
                    ('/tf','tf'),
                    ('/tf_static','tf_static'),
                    ('/odom','odom')
                    ]
        )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_namespace_cmd)
    # ld.add_action(log_param_change)
    ld.add_action(start_async_slam_toolbox_node)

    return ld