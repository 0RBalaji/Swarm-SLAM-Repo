import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "twowheelbot"
    namespace = LaunchConfiguration('namespace')
    pkg_path = os.path.join(get_package_share_directory(pkg_name),
                            'launch', 'bot_launch.py')
    gazebo_path = os.path.join(get_package_share_directory('gazebo_ros'),
                               'launch', 'gazebo.launch.py')
    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name),
                                      'config','gazebo_params.yaml')
    # world_file = os.path.join(get_package_share_directory(pkg_name),
    #                           'worlds', 'warehouse.world')

    pose = LaunchConfiguration('pose')

    pose = {
        'x': LaunchConfiguration('x_pose', default='2.5'),
        'y': LaunchConfiguration('y_pose', default='4'),
        'z': '0.05',
        'R': '0.00',
        'P': '0.00',
        'Y': '-3.142'
    }

    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_path]),
        launch_arguments = {'use_sim_time': 'true', 'namespace':namespace}.items()
    )
    
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([gazebo_path]),
    #     launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    # )
    
    spawn = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        namespace=namespace,
        arguments = ['-entity', namespace, '-topic','robot_description', '-entity', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
        output = 'screen'
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='pass the namespace with the _ at end'),
        DeclareLaunchArgument('x_pose', default_value='2.5', description='X position for the robot spawn'),
        DeclareLaunchArgument('y_pose', default_value='4', description='Y position for the robot spawn'),
        rsp,
        # gazebo,
        spawn
        # diff_drive_spawner,
        # joint_broad_spawner
    ])