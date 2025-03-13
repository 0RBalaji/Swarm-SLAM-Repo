import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = get_package_share_directory('slam_nav')
    launch_dir = os.path.join(pkg_path, 'launch')

    robot_pkg = LaunchConfiguration('robot_pkg')
    declare_robot_pkg_cmd = DeclareLaunchArgument(
        'robot_pkg',
        default_value = 'twowheelbot',
        description = 'Robot Package name'
    )
    
    robot_dir = FindPackageShare(robot_pkg)
    robot_launch_dir = PathJoinSubstitution([robot_dir, 'launch'])
    
    # Launch Configuration Variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = os.path.join(pkg_path, 'worlds', 'warehouse.world') # Updated world path
    # pose = {
    #     'x': LaunchConfiguration('x_pose', default='2.5'),
    #     'y': LaunchConfiguration('y_pose', default='4'),
    #     'z': LaunchConfiguration('z_pose', default='0.05'),
    #     'R': LaunchConfiguration('roll', default='0.00'),
    #     'P': LaunchConfiguration('pitch', default='0.00'),
    #     'Y': LaunchConfiguration('yaw', default='3.142')
    # }
    
    pose = {
        'x': LaunchConfiguration('x_pose', default='17.259089574451416'),
        'y': LaunchConfiguration('y_pose', default='-8.986688946733075'),
        'z': LaunchConfiguration('z_pose', default='0.045'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='-0.01370242341700195')
    }

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

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_path, 'ros2_map_map.yaml'),
        description='Full path to map file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Name of the robot')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',  # Default to True to launch the simulator
        description='Whether to start the simulator (Gazebo)'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',  # Default to False; set to True if you don't want to start the Gazebo client
        description='Whether to execute headless (without a GUI, only the Gazebo server)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Self rviz required or not'
    )

    # Robot State Publisher with namespace
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([robot_launch_dir,'robot_sim.launch.py'])),
        launch_arguments = {
            'use_sim_time': use_sim_time,
            'namespace':namespace,
            'use_namespace':use_namespace,
            'use_ros2_control':use_ros2_control,
            'use_simulator': use_simulator,
            'use_rviz':use_rviz,
            'x_pose': pose['x'],
            'y_pose': pose['y'],
            'z_pose': pose['z'],
            'roll': pose['R'],
            'pitch': pose['P'],
            'yaw': pose['Y'],}.items()
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file}.items()
    )

    # Launch Description
    ld = LaunchDescription()

    # Add launch options and commands
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_pkg_cmd)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add actions to launch the necessary nodes
    ld.add_action(robot)
    ld.add_action(bringup_cmd)

    return ld
