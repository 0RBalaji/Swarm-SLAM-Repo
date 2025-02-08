from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    remappings = [('/map_update_broadcast','map_update_broadcast'),
                    ('/map_merger_node','map_merger_node')]
    
    map_merger = Node(
        package='map_merger',
        executable='map_merger_node',
        name='decentralized_map_merger',
        namespace=namespace,
        output='screen',
        parameters=[
            {'namespace': namespace}
        ],
        remappings=remappings
    )

    map_update_broadcast = Node(
        package='map_merger',
        executable='map_update_broadcast',
        name='update_broadcast',
        namespace=namespace,
        output='screen',
        parameters=[
            {'namespace': namespace}
        ],
        remappings=remappings
    )

    return LaunchDescription([
        declare_namespace,
        map_merger,
        map_update_broadcast
    ])
