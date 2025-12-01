import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import xacro

def generate_launch_description():

    # Get the package directories
    pkg_aruco_visual_servoing = get_package_share_directory('aruco_visual_servoing')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Get the path to the URDF file
    robot_description_file = os.path.join(pkg_aruco_visual_servoing, 'urdf', 'aruco_bot_diff.xacro')

    # Parse the URDF file using xacro
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Start Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] # Pass the robot description to the node
    )

    # Include the Gazebo simulation launch file
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    # Include world file argument
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_aruco_visual_servoing, 'worlds', 'aruco_world.world'))

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
        arguments=['-d', os.path.join(pkg_aruco_visual_servoing, 'rviz', 'aruco_visual_servoing.rviz')],
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
            "-y", "0.0",
            "-z", "0.5"
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        rviz,
        spawn
    ])