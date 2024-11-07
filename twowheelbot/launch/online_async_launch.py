import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams, ReplaceString


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    default_params_file = os.path.join(get_package_share_directory("twowheelbot"),
                                       'config', 'mapper_params_online_async.yaml')

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
        description='Namespace for the slam_toolbox node')

    # Check if the provided param file has slam_toolbox params
    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    # Conditional parameter assignment based on namespace
    # if namespace != '':
    #     slam_toolbox_params = {
    #         'odom_frame': PythonExpression(['"', namespace, '/odom"']),
    #         'map_frame': PythonExpression(['"', namespace, '/map"']),
    #         # 'base_frame': PythonExpression(['"', namespace, '/base_footprint"']),
    #         # Add other parameters here if necessary
    #     }
    # else:
    #     slam_toolbox_params = {
    #         'odom_frame': 'odom',
    #         'map_frame': 'map',
    #         # 'base_frame': 'base_footprint',
    #         # Add other parameters here if necessary
    #     }

    replaced_params_file = ReplaceString(
        source_file=actual_params_file,
        replacements={'<robot_namespace>': ('/', namespace)})

    start_async_slam_toolbox_node = Node(
        parameters=[
            replaced_params_file,
            # slam_toolbox_params,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        remappings=[('/scan','scan'),
                    ('/map','map')]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(log_param_change)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
