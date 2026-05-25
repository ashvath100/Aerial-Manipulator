"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver_launch.py']),
            condition=IfCondition(LaunchConfiguration('server'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient_launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),        
    ])