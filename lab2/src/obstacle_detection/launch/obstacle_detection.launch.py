from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stop_distance_arg = DeclareLaunchArgument(
        "stop_distance",
        default_value="0.45",
        description="Distance in meters at which the robot reacts to obstacles",
    )

    goal_x_arg = DeclareLaunchArgument(
        "goal_x",
        default_value="2.0",
        description="Goal x position in odom coordinates",
    )

    goal_y_arg = DeclareLaunchArgument(
        "goal_y",
        default_value="1.0",
        description="Goal y position in odom coordinates",
    )

    stop_distance = LaunchConfiguration("stop_distance")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")

    obstacle_detection_cmd = Node(
        package="obstacle_detection",
        executable="obstacle_detection.py",
        name="obstacle_detection",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"stop_distance": stop_distance},
            {"goal_x": goal_x},
            {"goal_y": goal_y},
        ],
    )

    lidar_visualizer_cmd = Node(
        package="obstacle_detection",
        executable="lidar_visualizer.py",
        name="lidar_visualizer",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"stop_distance": stop_distance},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stop_distance_arg)
    ld.add_action(goal_x_arg)
    ld.add_action(goal_y_arg)

    ld.add_action(lidar_visualizer_cmd)

    # Starta obstacle_detection efter några sekunder så robot/scan/odom hinner komma igång
    ld.add_action(TimerAction(period=5.0, actions=[obstacle_detection_cmd]))

    return ld