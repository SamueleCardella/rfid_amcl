from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
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
        # Launch the Navigation2 Map Server (Lifecycle Node)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[parameters_file]
        ),
        # Launch the Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # or True, depending on your setup
                'autostart': True,
                'node_names': ['map_server']
            }]
        )
    ])
