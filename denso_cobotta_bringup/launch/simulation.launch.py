import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_denso_cobotta_moveit_config = get_package_share_directory('denso_cobotta_moveit_config')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_denso_cobotta_moveit_config, 'launch', 'demo.launch.py')))]
    )
