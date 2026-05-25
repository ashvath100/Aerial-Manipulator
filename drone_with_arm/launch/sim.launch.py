import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, EnvironmentVariable
from launch_ros.actions import Node

import random



def generate_launch_description():

    pkg_share = get_package_share_directory("drone_with_arm")
    urdf_file = os.path.join(pkg_share, "urdf", "two_dof_arm.urdf")
    controllers_file = os.path.join(pkg_share, "config", "controllers.yaml")

    # PX4 Gazebo enviroment setup
    # build and src path from home
    build_px4_gaz = '/PX4-Autopilot/build/px4_sitl_default'
    src_px4_gaz = '/PX4-Autopilot'

    # Environment variables for Gazebo
    extra_environment = [
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''), ':', os.environ['HOME'], build_px4_gaz, '/build_gazebo']),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), ':', os.environ['HOME'], src_px4_gaz, '/Tools/sitl_gazebo/models']),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=[EnvironmentVariable('LD_LIBRARY_PATH', default_value=''), ':', os.environ['HOME'], build_px4_gaz, '/build_gazebo'])
    ]

    # Robot State Publisher (TF only)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": open(urdf_file).read()}],
        output="screen",
    )

    world_file = "/home/ash/MAS_ws_public/src/drone_with_arm/worlds/optitrack.world"

    # Gazebo (classic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file  # must be a string
        }.items()
    )

        # Spawn drone with robot arm
    sdf_file = os.path.expanduser(
        "~/MAS_ws_public/src/drone_with_arm/models/sdu_drone_arm/sdu_drone_arm.sdf"
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "sdu_drone_arm",
            "-file", sdf_file,
            "-x", "0",
            "-y", "0",
            "-z", "0.5",
            "-R", "0",
            "-P", "0",
            "-Y", "0",
        ],
        output="screen",
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Arm controller
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file,
        ],
        output="screen",
    )

    # Gimbal controller
    gimbal_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gimbal_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_file,
        ],
        output="screen",
    )

    # spawn model by including px4_sitl_launch.py file and set it to be spawned.
    spawn_px4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "px4_sitl_launch.py")
        ), 
    )   


    # spawn mavros
    mavros_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node',
            '--ros-args',
            '--param', 'fcu_url:=udp://:14540@127.0.0.1:14557',
            '--param', 'plugin_denylist:=[odometry]'
        ],
        output='screen',
    )


    def spawn_target(name):
        x = random.uniform(-4, 4)
        y = random.uniform(-4, 4)
        z = random.uniform(-2.0, -0.5)

        return Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", name,
                "-database", "target_generic",
                "-x", str(x),
                "-y", str(y),
                "-z", str(z),
                "-robot_namespace", name
            ],
            output="screen",
        )

    # Create multiple target spawners
    targets = [spawn_target(f"target{i}") for i in range(1, 5)]

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        arm_controller,
        gimbal_controller,
        spawn_px4,
        mavros_node,
        *targets
    ])

