from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="map_loader",
            executable="mission_loader_node"
        ),
        Node(
            package="floodfill_pkg",
            executable="floodfill_node"
        )
    ])
