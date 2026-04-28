#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # --- Paths ---
    bringup_dir = get_package_share_directory("bringup")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    nav2_bt_navigator_dir = get_package_share_directory("nav2_bt_navigator")
    obstacle_dir = get_package_share_directory("obstacle_detection")
    # Default paths for files
    default_rviz_config = os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz")
    default_nav_params = os.path.join(bringup_dir, "params", "nav2_params.yaml")
    urdf_file = os.path.join(bringup_dir, "urdf", "turtlebot3_burger.urdf")
    world_file = os.path.join(bringup_dir, "worlds", "turtlebot3_world.world")
    map_file = os.path.join(bringup_dir, "maps", "map.yaml")
    default_bt_xml = os.path.join(
        nav2_bt_navigator_dir,
        "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml",
    )
    burger_sdf = os.path.join(bringup_dir, "models", "turtlebot3_burger", "model.sdf")
    bridge_params = os.path.join(bringup_dir, "params", "turtlebot3_burger_bridge.yaml")

    # Read URDF file content
    with open(urdf_file, "r") as infp:
        robot_description_content = infp.read()

    # --- Launch Configurations ---
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    enable_rviz = LaunchConfiguration("enable_rviz", default="True")
    rviz_config_file = LaunchConfiguration(
        "rviz_config_file", default=default_rviz_config
    )
    params_file = LaunchConfiguration("nav_params_file", default=default_nav_params)
    use_simulator = LaunchConfiguration("use_simulator", default="True")
    headless = LaunchConfiguration("headless", default="False")
    # --- Launch Arguments Declarations ---
    declare_use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", default_value="True", description="Use simulator time"
    )
    declare_enable_rviz_arg = DeclareLaunchArgument(
        name="enable_rviz", default_value="True", description="Enable rviz launch"
    )
    declare_rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_config,
        description="Full path to the RVIZ config file to use",
    )
    declare_params_file_arg = DeclareLaunchArgument(
        "nav_params_file",
        default_value=default_nav_params,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator",
        default_value="True",
        description="Whether to start the simulator",
    )

    declare_headless_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )
    # --- Nodes and Launch Includes ---
    # Gazebo
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r -s -v2 ", world_file], "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(use_simulator),
    )
    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v2 ", "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(PythonExpression([use_simulator, " and not ", headless])),
    )

    start_obstacle_det_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(obstacle_dir, "launch", "obstacle_detection.launch.py")
        )
    )

    # Common Remappings
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Map Server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_file}],
        remappings=remappings,
    )
    map_server_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "publish_frequency": 10.0,
                "robot_description": robot_description_content,
            }
        ],
        remappings=remappings,
    )

    # Spawn Turtlebot3 Burger
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            burger_sdf,
            "-name",
            "turtlebot3_burger",
            "-x",
            "-1.5",
            "-y",
            "-0.5",
            "-z",
            "0.01",
            "-Y",
            "0.0",
        ],
        output="screen",
        condition=IfCondition(use_simulator),
    )

    # ROS-GZ Bridge
    gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": bridge_params,
            "expand_gz_topic_names": False,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # Navigation2 Bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "use_namespace": "False",
            "map": "",  # Using map_server declared above
            "map_server": "False",  # Don't run internal map server
            "params_file": params_file,
            "default_bt_xml_filename": default_bt_xml,
            "autostart": "True",
            "use_sim_time": use_sim_time,
            "log_level": "warn",
        }.items(),
    )

    # RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rviz_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_namespace": "False",
            "rviz_config": rviz_config_file,
            "log_level": "warn",
        }.items(),
        condition=IfCondition(enable_rviz),
    )

    # --- Event Handlers ---
    actions_on_spawn_exit = [
        robot_state_publisher_node,
        nav2_bringup_cmd,
    ]

    register_core_on_spawn_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node, on_exit=actions_on_spawn_exit
        ),
        condition=IfCondition(use_simulator),
    )

    rviz_timer = TimerAction(period=5.0, actions=[rviz_cmd])

    register_rviz_headless_on_spawn_exit = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_robot_node, on_exit=[rviz_cmd]),
        condition=IfCondition(
            PythonExpression([use_simulator, " and ", headless, " and ", enable_rviz])
        ),
    )

    register_rviz_timer_on_spawn_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node, on_exit=[rviz_timer]
        ),
        condition=IfCondition(
            PythonExpression(
                [use_simulator, " and not ", headless, " and ", enable_rviz]
            )
        ),
    )

    # --- Add Actions to Launch Description ---
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_enable_rviz_arg)
    ld.add_action(declare_rviz_config_file_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_obstacle_det_cmd)

    ld.add_action(map_server_node)
    ld.add_action(map_server_lifecycle_node)

    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(spawn_robot_node)

    ld.add_action(register_core_on_spawn_exit)
    ld.add_action(register_rviz_headless_on_spawn_exit)
    ld.add_action(register_rviz_timer_on_spawn_exit)

    return ld
