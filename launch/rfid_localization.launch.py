from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the shared directory of the package
    package_share_directory = get_package_share_directory('rfid_amcl_node')
    parameters_file = os.path.join(package_share_directory, 'params', 'rfid_localization_params.yaml')

    return LaunchDescription([
        # Launch the RFID AMCL node
        Node(
            package='rfid_amcl_node',
            executable='rfid_amcl_node',
            name='rfid_amcl',
            output='screen',
            parameters=[parameters_file]
        ),
        # Launch the Navigation2 Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[parameters_file]
        )
    ])