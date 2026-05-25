
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Demo for spawn_entity.
Launches Gazebo and spawns a model
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    ## Gazebo enviroment setup
    # build and src path from home
    build_px4_gaz = '/PX4-Autopilot/build/px4_sitl_default'
    src_px4_gaz = '/PX4-Autopilot'

    # FOXY does not have the AppendEnvironmentVariable action, and thus we have to go verbose
    extra_environment = [
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''), ':', os.environ['HOME'], build_px4_gaz, '/build_gazebo']),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), ':', os.environ['HOME'], src_px4_gaz, '/Tools/sitl_gazebo/models']),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=[EnvironmentVariable('LD_LIBRARY_PATH', default_value=''), ':', os.environ['HOME'], build_px4_gaz, '/build_gazebo'])
    ]

    DeclareLaunchArgument('vehicle', default_value='sdu_drone',
                          description='Set the vehicle to be spawned'),     

    # spawn model by including px4_sitl_launch.py file and set it to be spawned.
    spawn_px4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/px4_sitl_launch.py']),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-database', LaunchConfiguration('vehicle'), '-entity', LaunchConfiguration('vehicle'), '-x', '0', '-y', '0', '-z', '1.0', '-R', '0', '-P', '0', '-Y', '0'],
        output='screen')

    # spawn mavros
    spawn_mavros = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([ThisLaunchFileDir(), '/mavros_launch.xml']),
    )

    return LaunchDescription(extra_environment + [
        spawn_px4,
        spawn_entity
    ])
