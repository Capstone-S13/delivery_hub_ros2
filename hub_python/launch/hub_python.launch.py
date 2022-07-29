from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    cnc_params = os.path.join(
        get_package_share_directory('cnc_interface'),
        'params.yaml'
    )

    toolhead_params = os.path.join(
        get_package_share_directory('toolhead_action'),
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='toolhead_action',
            executable='toolhead_action_server',
            name='toolhead_action_server_node',
            parameters=[toolhead_params]
        ),

        Node(
            package='cnc_interface',
            executable='cnc_interface_node',
            name='cnc_interface_node',
            parameters=[cnc_params]
        ),
        Node(
            package='hub_python',
            executable='hub_node',
            name='hub_node',
        ),

    ])