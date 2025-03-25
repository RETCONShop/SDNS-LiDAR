from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vn200_node = Node(
            package="snl_vn200",
            executable="snl_vn200",
            name="snl_vn200",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    lidar_node = Node(
            package="snl_lidarcollector",
            executable="snl_lidarnode",
            name="snl_lidarnode",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    
    odom_node = Node(
            package="snl_odomnode",
            executable="snl_odomnode",
            name="snl_odomnode",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    dataout_node = Node(
            package="snl_dataout",
            executable="snl_dataout",
            name="snl_dataout",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    commandin_node = Node(
            package="snl_commandin",
            executable="snl_commandin",
            name="snl_commandin",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    mech_node =  Node(
            package="snl_mechanization",
            executable="snl_mechanization",
            name="snl_mechanization",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    filter_node = Node(
            package="snl_kalmanfilter",
            executable="snl_kalmanfilter",
            name="snl_kalmanfilter",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    movementcontrol_node = Node(
            package="snl_movementcontrol",
            executable="snl_movementcontrol",
            name="snl_movementcontrol",
            output="log",
            emulate_tty=False,
            namespace="bot1"
        )
    return LaunchDescription([
        vn200_node,
        lidar_node,
        odom_node,
        dataout_node,
        commandin_node,
        mech_node,
        filter_node,
        movementcontrol_node
    ])