import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'warehouse_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Include the Simulation Launch (Gazebo + Robot + Bridge)
    # This reuses the logic from your existing launch_sim.launch.py
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_sim.launch.py')
        )
    )

    # 2. Launch RViz with our custom configuration
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'warehouse.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim_launch,
        rviz_node
    ])