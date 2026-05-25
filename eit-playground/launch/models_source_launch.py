

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gaz_pkg_share = get_package_share_directory('eit_playground')

    # source models etc.
    cmd = [[
        '. ',
        gaz_pkg_share,
        '/setup_gazebo.bash'
    ]]

    return LaunchDescription([
        ExecuteProcess(
            cmd=cmd,
            output='screen',
            shell=True,
        )
    ])
