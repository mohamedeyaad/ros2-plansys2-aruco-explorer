import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Get the package directories
    pkg_arucobot_navigation = get_package_share_directory('arucobot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_arucobot_gazebo = get_package_share_directory('arucobot_gazebo')
    
    # Files
    nav2_params_file = os.path.join(pkg_arucobot_navigation, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_arucobot_navigation, 'maps', 'my_map.yaml') # It should matche what the name of the map!
    # 1. Launch the Simulation & Bridge (existing launch file)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_arucobot_gazebo, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'False'  # Disable RViz in the simulation launch
        }.items()
    )

    # 2. Launch Nav2 Bringup (Localization + Path Planning)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_file,
            'params_file': nav2_params_file
        }.items()
    )

    # 3. Launch RViz (Pre-configured for Nav2)
    rviz_config = os.path.join(pkg_arucobot_navigation, 'rviz', 'arucobot_navigation.rviz')
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