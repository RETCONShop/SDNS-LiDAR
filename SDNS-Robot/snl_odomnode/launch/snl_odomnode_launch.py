from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="snl_odomnode",
            executable="snl_odomnode",
            name="snl_odomnode",
            output="log",
            emulate_tty=False,
            parameters=[
                {"enabled": True},
            ]
        )
    ])