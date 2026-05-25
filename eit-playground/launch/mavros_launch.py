
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
    # spawn mavros
    spawn_mavros = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([ThisLaunchFileDir(), '/mavros_launch.xml']),
    )

    return LaunchDescription([
        spawn_mavros
    ])
