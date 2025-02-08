from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_file = os.path.join(get_package_share_directory('map_merger'), 'config', 'merge_map.rviz')

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='map_merger',
            executable='merge_map_node',
            name='merge_map_node',
            parameters=[{'use_sim_time': True}, {'namespace': namespace}],
            namespace=namespace,
            # remappings=[
            # ('/tf', 'tf'),
            # ('/tf_static', 'tf_static'),
            # ('/map','map'),
            # ('/map_merge','map_merge'),
            # ]
        )
    ])