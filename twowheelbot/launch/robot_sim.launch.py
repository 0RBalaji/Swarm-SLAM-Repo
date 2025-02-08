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

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = os.path.join(pkg_path, 'worlds', 'warehouse.world')

    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pose = {
        'x': LaunchConfiguration('x_pose', default='2.5'),
        'y': LaunchConfiguration('y_pose', default='4'),
        'z': LaunchConfiguration('z_pose', default='0.05'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='-3.142')
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel'),
        ('/odom','odom'),
        ('/map','map')]

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
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Self rviz required or not'
    )
    
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

    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='False',
        description='Hardware control. By Default set to simulation'
    )

    gazebo_params_file = os.path.join(pkg_path, 'config','gazebo_params.yaml')
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,'rsp.launch.py')),
        launch_arguments = {'use_sim_time': use_sim_time, 'namespace':namespace, 'use_namespace':use_namespace, 'use_ros2_control':use_ros2_control, 'use_rviz':use_rviz}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items()
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

    twist_mux_params = os.path.join(pkg_path, 'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        condition = IfCondition(use_ros2_control),
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    # ros2_ctrl_params = os.path.join(pkg_path, 'config', 'my_controllers.yaml')
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["diff_cont"],
        condition = IfCondition(use_ros2_control),
        remappings=remappings
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_broad"],
        condition = IfCondition(use_ros2_control),
        remappings=remappings
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(rsp)
    ld.add_action(gazebo)
    
    ld.add_action(spawn)
    ld.add_action(twist_mux)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    return ld