import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_params_file = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'camera_calibration.yaml'
    )

    aruco_params_file = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'aruco_markers.yaml'
    )

    # Uruchomienie węzła publikującego obraz
    camera_node = Node(
        package='camera_pkg',
        executable='camera_publisher',
        name='oak_d_camera_node'
    )

    # Uruchomienie węzła detekcji ArUco
    aruco_node = Node(
        package='camera_pkg',
        executable='aruco_detector',
        name='aruco_detector_node',
        output='screen',
        parameters=[camera_params_file, aruco_params_file]
    )

    return LaunchDescription([
        camera_node,
        aruco_node
    ])