import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = get_package_share_directory('twowheelbot')
    launch_dir = os.path.join(pkg_path, 'launch')
    
    # Launch Configuration Variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = os.path.join(pkg_path, 'worlds', 'warehouse.world') # Updated world path
    pose = {
        'x': LaunchConfiguration('x_pose', default='2.5'),
        'y': LaunchConfiguration('y_pose', default='4'),
        'z': LaunchConfiguration('z_pose', default='0.05'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='-3.142')
    }
    
    robot_name = LaunchConfiguration('robot_name')
    urdf_file = os.path.join(pkg_path, 'designs', 'amr.urdf.xacro')

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel')]

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

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_path, 'maps', 'ros2_map_map.yaml'),
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
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            pkg_path, 'config', 'viz.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Self rviz required or not'
    )

    # Read the URDF
    robot_description = Command(['xacro ', urdf_file, ' robot_namespace:=', namespace, ' sim_mode:=', use_sim_time])
    # print(f"Using URDF file: {urdf_file}")
    
    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
    )

    # Robot State Publisher with namespace
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

    
    if use_namespace == True:
        robot_description_topic = [TextSubstitution(text='/'), LaunchConfiguration('namespace'), TextSubstitution(text='/robot_description')]
    else:
        robot_description_topic = TextSubstitution(text='robot_description')
    # robot_description_topic = [LaunchConfiguration('namespace'), TextSubstitution(text='/robot_description')]

    # Spawn robot in Gazebo with namespace
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace,
        output='screen',
        arguments=[
            '-topic', robot_description_topic,
            '-entity', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']
            ]
    )

    # RViz Launch with namespace
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'rviz_config': rviz_config_file,
                          'robot_description': robot_description}.items()
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
    ld.add_action(declare_use_sim_time_cmd)         ### For other file
    ld.add_action(declare_map_yaml_cmd)            ### For other file
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_params_file_cmd)         ### For other file
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(declare_rviz_config_file_cmd)

    # Add actions to launch the necessary nodes
    ld.add_action(rsp)
    ld.add_action(jsp)
    # ld.add_action(gazebo)
    
    ld.add_action(spawn)
    ld.add_action(bringup_cmd)                   ### For other file
    ld.add_action(rviz_cmd)

    return ld
