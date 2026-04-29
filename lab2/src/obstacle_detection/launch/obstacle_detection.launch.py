#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stop_distance = LaunchConfiguration("stop_distance")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")

    return LaunchDescription([
        DeclareLaunchArgument(
            "stop_distance",
            default_value="0.45",
            description="Obstacle stop distance",
        ),
        DeclareLaunchArgument(
            "goal_x",
            default_value="2.0",
            description="Goal x position",
        ),
        DeclareLaunchArgument(
            "goal_y",
            default_value="0.0",
            description="Goal y position",
        ),

        Node(
            package="obstacle_detection",
            executable="obstacle_detection.py",
            name="obstacle_detection",
            output="screen",
            parameters=[
                {
                    "stop_distance": stop_distance,
                    "goal_x": goal_x,
                    "goal_y": goal_y,
                }
            ],
        ),

        Node(
            package="obstacle_detection",
            executable="lidar_visualizer.py",
            name="lidar_visualizer",
            output="screen",
            parameters=[
                {
                    "stop_distance": stop_distance,
                }
            ],
        ),
    ])