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
import launch

def generate_launch_description():
    # Get the launch directory
    pkg_path = get_package_share_directory('twowheelbot')
    launch_dir = os.path.join(pkg_path, 'launch')

    # Names and poses of the robots
    robots = [
        {'name': 'botA', 'x_pose': 3.0, 'y_pose': 2.5, 'z_pose': 0.045,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': -3.142},
        {'name': 'botB', 'x_pose': 3.0, 'y_pose': 5.5, 'z_pose': 0.045,
                           'roll': 0.0, 'pitch': 0.0, 'yaw': -3.142}
        #                    ,
        # {'name': 'robotC', 'x_pose': 4.5, 'y_pose': 1.0, 'z_pose': 0.045,
        #                    'roll': 0.0, 'pitch': 0.0, 'yaw': -3.142},
        # {'name': 'robotD', 'x_pose': 4.5, 'y_pose': 7, 'z_pose': 0.045,
        #                    'roll': 0.0, 'pitch': 0.0, 'yaw': -3.142}
        ]


    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    # autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
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

    declare_robotA_params_file_cmd = DeclareLaunchArgument(
        'robotA_params_file',
        default_value=os.path.join(pkg_path, 'params', 'nav2_multirobot_params_A.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_robotB_params_file_cmd = DeclareLaunchArgument(
        'robotB_params_file',
        default_value=os.path.join(pkg_path, 'params', 'nav2_multirobot_params_B.yaml'),
        description='Full path to the ROS2 parameters file to use for robot2 launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Automatically startup the stacks',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_path, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.',
    )

    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     'rviz_config',
    #     default_value=os.path.join(pkg_path, 'config', 'viz.rviz'),
    #     description='Full path to the RVIZ config file to use.',
    # )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')


    # declare_gui_cmd = DeclareLaunchArgument(
    #     'gui',
    #     default_value='true',
    #     description='Whether to start Gazebo with GUI'
    # )

    # Start Gazebo with plugin providing the robot spawning service
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
    #                                  '-s', 'libgazebo_ros_factory.so', world],
    #     output='screen')
    
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        'namespace': TextSubstitution(text=robot['name']),
                        'use_namespace': 'True',
                        'rviz_config': rviz_config_file
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_path, 'launch', 'multi_launch.py')
                    ),
                    launch_arguments={
                        'namespace': robot['name'],
                        'use_namespace': 'True',
                        'map': map_yaml_file,
                        'use_sim_time': 'True',
                        'params_file': params_file,
                        # 'autostart': autostart,
                        'use_rviz': 'False',
                        'use_simulator': 'False',
                        'headless': 'False',
                        'use_robot_state_pub': use_robot_state_pub,
                        'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                        'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                        'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                        'roll': TextSubstitution(text=str(robot['roll'])),
                        'pitch': TextSubstitution(text=str(robot['pitch'])),
                        'yaw': TextSubstitution(text=str(robot['yaw'])),
                        'robot_name': TextSubstitution(text=robot['name']), }.items(),
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
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'], ' rviz config file: ', rviz_config_file],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name'],' using robot state pub: ',use_robot_state_pub,],
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot['name']],                ####, ' autostart: ', autostart
                ),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    # ld.add_action(declare_robotA_params_file_cmd)
    # ld.add_action(declare_robotB_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld