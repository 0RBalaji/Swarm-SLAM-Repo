import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = get_package_share_directory('amr_bot')
    launch_dir = os.path.join(pkg_path, 'launch')
    
    # Launch Configuration Variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_rviz = LaunchConfiguration('use_rviz')

    
    urdf_file = os.path.join(pkg_path, 'designs', 'amr.urdf.xacro')

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel'),
        ('/odom','odom'),
        ('/map','map')]

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if True')
    
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='False',
        description='Hardware control. By Default set to simulation'
    )
        
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Self rviz required or not'
    )

    rviz_config_file = PythonExpression([
        '\"', os.path.join(pkg_path, 'rviz', 'default_view.rviz'), '\" if not ', use_namespace,
        ' else \"', os.path.join(pkg_path, 'rviz', 'namespaced_view.rviz'), '\"'
    ])


    # Read the URDF
    # robot_description = Command(['xacro ', urdf_file, ' robot_namespace:=', namespace, ' sim_mode:=', use_sim_time])
    robot_description = Command(['xacro ', urdf_file, ' robot_namespace:=', namespace, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings
    )
    
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                        'rviz_config': rviz_config_file,
                        'robot_description': robot_description}.items()
    )    

    # Launch Description
    ld = LaunchDescription()

    # Add launch options and commands
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(declare_use_rviz_cmd)

    # Add actions to launch the necessary nodes
    ld.add_action(rsp)
    ld.add_action(jsp)
    ld.add_action(rviz_cmd)

    return ld