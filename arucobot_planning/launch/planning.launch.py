import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the package directory 
    pkg_arucobot_navigation = get_package_share_directory('arucobot_navigation')
    pkg_arucobot_planning = get_package_share_directory('arucobot_planning')
    pkg_plansys2_bringup = get_package_share_directory('plansys2_bringup')
    
    # 1. Path to Domain & Problem File
    domain_file = os.path.join(pkg_arucobot_planning, 'pddl', 'domain.pddl')
    problem_file = os.path.join(pkg_arucobot_planning, 'pddl', 'problem.pddl') 

    # 2. Launch PlanSys2
    plansys2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_plansys2_bringup, 'launch', 'plansys2_bringup_launch_distributed.py')
        ),
        launch_arguments={
            'model_file': domain_file,
            'problem_file': problem_file,
        }.items()
    )

    # 3. Launch Move Action Node
    move_action_node = Node(
        package='arucobot_planning',
        executable='move_action_node', # Name of the executable in CMakeLists.txt
        name='move_action_node',
        output='screen',
    )
    
    return LaunchDescription([
        plansys2_launch,
        move_action_node
    ])