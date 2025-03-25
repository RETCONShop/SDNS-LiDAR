from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="snl_startup",
            executable="snl_startup",
            name="snl_startup",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    ])