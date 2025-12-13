import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('aruco_visual_servoing'),
        'config',
        'aruco_params.yaml'
        )

    aruco_node = Node(
        package='aruco_visual_servoing',
        executable='aruco_detector_node', # or whatever you named the entry point
        parameters=[aruco_params]
    )
    visual_servoing_node = Node(
            package='aruco_visual_servoing',
            executable='aruco_visual_servoing_node', # or whatever you named the entry point
            name='sequential_aruco_chaser'
        )
    return LaunchDescription([
        aruco_node,
        visual_servoing_node
    ])
