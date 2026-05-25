"""Launch Gazebo, RVIZ2 and SGRE Windturbine"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.substitutions import ThisLaunchFileDir, EnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.event_handlers import OnStateTransition
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    ## Gazebo enviroment setup
    # build and src path from home
    build_px4_gaz = '$PX4_DIR/build/px4_sitl_default'
    src_px4_gaz = '$PX4_DIR'

    # FOXY does not have the AppendEnvironmentVariable action, and thus we have to go verbose
    extra_environment = [
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''), ':', os.environ['HOME'], build_px4_gaz, '/build_gazebo']),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), ':', os.environ['HOME'], src_px4_gaz, '/Tools/sitl_gazebo/models']),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=[EnvironmentVariable('LD_LIBRARY_PATH', default_value=''), ':', os.environ['HOME'], build_px4_gaz, '/build_gazebo'])
    ]

    # start gazebo, then spawn the wind turbine in world
    gaz_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/gazebo_launch.py']
        ),
    )

    # actions - Start nodes in right order
    nodes = [
        gaz_start
    ]

    return LaunchDescription(extra_environment + declared_arguments + nodes)
