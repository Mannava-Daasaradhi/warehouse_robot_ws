import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_share = get_package_share_directory('warehouse_robot')
    
    # Path to our config file
    param_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    # Include the official slam_toolbox launch file
    # This handles the Lifecycle Nodes automatically!
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': param_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        slam_launch
    ])