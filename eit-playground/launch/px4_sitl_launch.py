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

"""Launch a Gazebo client with command line arguments."""

from os import environ
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory

from scripts import GazeboRosPaths


def generate_launch_description():
    cmd = [[
        '$PX4_DIR/build/px4_sitl_default/bin/px4 ',
        '$PX4_DIR/ROMFS/px4fmu_common ',
        '-s $PX4_DIR/build/px4_sitl_default/etc/init.d-posix/rcS ',
        '-i ',
        LaunchConfiguration('id'),
        ' -w ',
        LaunchConfiguration('working_dir')
    ]]

    env = {
        'PX4_SIM_MODEL': LaunchConfiguration('model')
    }

    # get share path
    gaz_pkg_share = get_package_share_directory('eit_playground')
    default_working_dir = LaunchConfiguration('model')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='sdu_drone',
                            description='Set the model used for the px4 instance'),

        DeclareLaunchArgument('id', default_value='0',
                                description='id, set the id used for the instance, first model spawned == 0'),

        # change so default value is not sitl_laerke in top-level repo, but moved into a config folder... Get it into the share namespace
        DeclareLaunchArgument('working_dir', default_value=PathJoinSubstitution([gaz_pkg_share, default_working_dir]),
                                description='working directory for saved settings and configurations. Use the same name for same simulations'),

        # Execute node with parameters
        ExecuteProcess(
            cmd=cmd,
            output='screen',
            additional_env=env,
            shell=True,
        ),

    ])
