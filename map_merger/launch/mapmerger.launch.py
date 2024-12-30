from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    remappings = [('/map_update_broadcast','map_update_broadcast'),
                    ('/map_merging_node','map_merging_node')]

    map_merger = Node(
        package='map_merger',
        executable='map_merger_node',
        name='map_merger_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {'namespace': namespace}
        ],
        remappings=remappings
    )
    
    new_map_merger = Node(
        package='map_merger',
        executable='map_merging_node',
        name='map_merging',
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
        
        # map_merger,
        new_map_merger,
        map_update_broadcast
    ])
