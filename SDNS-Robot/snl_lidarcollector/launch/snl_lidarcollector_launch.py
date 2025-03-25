from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="snl_lidarcollector",
            executable="snl_lidarnode",
            name="snl_lidarnode",
            output="log",
            emulate_tty=False,
            parameters=[
                {"scan_interval": 2000},
                {"enabled": True},
                {"max_distance": 1000.0}
            ]
        )
    ])