import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    pkg_path = get_package_share_directory('slam_nav')
    # merge_path = get_package_share_directory('map_merger')
    launch_dir = os.path.join(pkg_path, 'launch')

    # rmf_dir = os.path.join(get_package_share_directory('rmf_pro'))

    robot_pkg = LaunchConfiguration('robot_pkg')

    declare_robot_pkg_cmd = DeclareLaunchArgument(
        'robot_pkg',
        default_value = 'twowheelbot',
        description = 'Robot Package  name'
    )

    # Names and poses of the robots
    # robots = [
    #     {'name': 'botA', 'x_pose': 6.2, 'y_pose': 2.2, 'z_pose': 0.01,
    #                        'roll': 0.0, 'pitch': 0.0, 'yaw': -3.142},
    #     {'name': 'botB', 'x_pose': 1.9, 'y_pose': 9.7, 'z_pose': 0.01,
    #                        'roll': 0.0, 'pitch': 0.0, 'yaw': -1.57},
    #     {'name': 'botC', 'x_pose': -3.6, 'y_pose': 9.7, 'z_pose': 0.01,
    #                        'roll': 0.0, 'pitch': 0.0, 'yaw': -1.57},
    #     {'name': 'botD', 'x_pose': -3.6, 'y_pose': -9.6, 'z_pose': 0.01            ,
    #                        'roll': 0.0, 'pitch': 0.0, 'yaw': 1.57}
    # ]

    robots = [
        # {'name': 'botA', 'x_pose': 17.259089574451416, 'y_pose': -8.986688946733075, 'z_pose': 0.045,
        #                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        # {'name': 'botB', 'x_pose': 17.00991550716311, 'y_pose': -3.0730820241987633, 'z_pose': 0.045,
        #                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'botA', 'x_pose': 4.5, 'y_pose': 1.0, 'z_pose': 0.045,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        # {'name': 'robotD', 'x_pose': 4.5, 'y_pose': 7, 'z_pose': 0.045,
        #                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    ]


    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    # autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_path, 'worlds', 'warehouse.world'),
        description='Full path to world file to load',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_path, 'ros2_map_map.yaml'),
        description='Full path to map file to load',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Automatically startup the stacks',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='False',
        description='For hardware IG!',
    )

    # Start Gazebo with plugin providing the robot spawning service
    """
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')
    """

    start_gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    start_gz_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_path, 'launch', 'amr_launch.py')
                    ),
                    launch_arguments={
                        'namespace': robot['name'],
                        'use_namespace': 'True',
                        'map': map_yaml_file,
                        'use_sim_time': 'True',
                        'params_file': params_file,
                        # 'autostart': autostart,
                        'use_rviz': use_rviz,
                        'use_simulator': 'False',
                        'robot_pkg' : robot_pkg,
                        'use_ros2_control':use_ros2_control,
                        'headless': 'False',
                        'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                        'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                        'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                        'roll': TextSubstitution(text=str(robot['roll'])),
                        'pitch': TextSubstitution(text=str(robot['pitch'])),
                        'yaw': TextSubstitution(text=str(robot['yaw'])),}.items(),
                ),

                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=['Launching ', robot['name']],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' map yaml: ', map_yaml_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' params yaml: ', params_file],
                ),
                # LogInfo(
                #     condition=IfCondition(log_settings),
                #     msg=[robot['name']], ' autostart: ', autostart
                # ),
            ]
        )

        nav_instances_cmds.append(group)
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_pkg_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_ros2_control_cmd)

    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_gz_client_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld