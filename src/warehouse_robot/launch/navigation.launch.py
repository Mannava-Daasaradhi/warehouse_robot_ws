import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_robot')
    
    # 1. Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file_path = os.path.join(pkg_share, 'maps', 'warehouse_map.yaml')
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 2. Launch Nav2 Bringup (The "Brain")
    # This launches AMCL (Localization) and the Planner/Controller
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file_path,
            'params_file': params_file_path,
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])