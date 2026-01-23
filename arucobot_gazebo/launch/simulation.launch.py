import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import xacro

def generate_launch_description():

    # Get the package directories
    pkg_arucobot_description = get_package_share_directory('arucobot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_arucobot_gazebo = get_package_share_directory('arucobot_gazebo')

    # Get the path to the URDF file
    robot_description_file = os.path.join(pkg_arucobot_description, 'urdf', 'robot.xacro')
    # Parse the URDF file using xacro
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Declare launch arguments to toggle RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', 
        default_value='true', 
        description='Open RViz inside this simulation'
    )

    # Start Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}] # Pass the robot description to the node
    )

    # Include the Gazebo simulation launch file
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    # Include world file argument
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_arucobot_gazebo, 'worlds', 'aruco_world.world'))

    # Start Gazebo simulation with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    # Start RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_arucobot_gazebo, 'rviz', 'arucobot_gazebo.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Spawn Robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            # Topic to read robot description from where the robot_state_publisher is publishing and when to spawn the robot in Gazebo
            "-topic", "/robot_description",
            "-name", "aruco_bot",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "1.0",
            "-z", "1.2"
        ]
    )

    # Get the path to the ROS-GZ bridge config file
    ros_gz_bridge_config = os.path.join(
        pkg_arucobot_gazebo,
        'config',
        'ros_gz_bridge.yaml'
    )

    # Bridge ROS and Gazebo topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config,
            'use_sim_time': True
        }],
        output='screen'
    )

    # EKF Node for state estimation
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_arucobot_gazebo, 'config', 'ekf.yaml'),
                    {'use_sim_time': True}]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher,
        gazebo,
        rviz,
        spawn, 
        bridge,
        ekf_node
    ])