import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'arucobot_navigation'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Files
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml') # It should matche what the name of the map!

    # 1. Launch the Simulation & Bridge (existing launch file)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arucobot_gazebo'), 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'False'  # Disable RViz in the simulation launch
        }.items()
    )

    # 2. Launch Nav2 Bringup (Localization + Path Planning)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file,
            'params_file': nav2_params_file
        }.items()
    )

    # 3. Launch RViz (Pre-configured for Nav2)
    # Use the official Nav2 rviz config so no need to setup displays manually
    rviz_config = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        nav2_launch,
        rviz_node
    ])