"""Run the deterministic simulator with independent robot map access."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("semaforr"))
    config_dir = package_share / "config"
    example_dir = config_dir / "example"
    default_output = (
        Path.home() / ".ros" / "semaforr" / "example-simulation.json"
    )

    output = LaunchConfiguration("output")
    duration = LaunchConfiguration("duration")
    sensor_cutoff = LaunchConfiguration("sensor_cutoff")
    startup_delay = LaunchConfiguration("startup_delay")
    use_rviz = LaunchConfiguration("rviz")
    simulator_map = LaunchConfiguration("simulator_environment_map")
    robot_map_mode = LaunchConfiguration("semaforr_map_mode")
    robot_map_path = LaunchConfiguration("semaforr_map_path")
    robot_map_planning = LaunchConfiguration("map_based_planning")
    robot_map_visualization = LaunchConfiguration("map_visualization")
    map_planners = LaunchConfiguration("map_planners")

    def launch_nodes(context):
        planner_names = [
            value.strip()
            for value in map_planners.perform(context).split(",")
            if value.strip()
        ]
        semaforr = Node(
            package="semaforr",
            executable="semaforr_node",
            name="semaforr",
            output="screen",
            parameters=[
                str(config_dir / "semaforr.yaml"),
                {
                    "map.mode": robot_map_mode.perform(context),
                    "map.path": robot_map_path.perform(context),
                    "map.planning.enabled": ParameterValue(
                        robot_map_planning, value_type=bool
                    ),
                    "map.visualizations.enabled": ParameterValue(
                        robot_map_visualization, value_type=bool
                    ),
                    "map.length_m": 12,
                    "map.height_m": 12,
                    "mission.tasks_path": str(example_dir / "mission.conf"),
                    "planners.enabled": planner_names,
                },
            ],
        )
        simulator_arguments = [
            "--output",
            output,
            "--duration",
            duration,
            "--sensor-cutoff",
            sensor_cutoff,
            "--initial-x",
            "2.0",
            "--initial-y",
            "2.0",
            "--scenario-name",
            "example_open_room",
        ]
        selected_simulator_map = simulator_map.perform(context)
        if selected_simulator_map:
            simulator_arguments.extend(
                ["--environment-map", selected_simulator_map]
            )
        simulator = Node(
            package="semaforr",
            executable="semaforr_record_baseline",
            name="semaforr_example_simulator",
            output="screen",
            arguments=simulator_arguments,
        )
        stop_when_complete = RegisterEventHandler(
            OnProcessExit(
                target_action=simulator,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                "The deterministic SemaFORR example "
                                "completed"
                            )
                        )
                    )
                ],
            )
        )
        return [
            semaforr,
            TimerAction(period=startup_delay, actions=[simulator]),
            stop_when_complete,
        ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="semaforr_rviz",
        output="screen",
        arguments=["-d", str(package_share / "rviz" / "semaforr.rviz")],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("output", default_value=str(default_output)),
            DeclareLaunchArgument("duration", default_value="20.0"),
            DeclareLaunchArgument("sensor_cutoff", default_value="-1.0"),
            DeclareLaunchArgument("startup_delay", default_value="3.0"),
            DeclareLaunchArgument(
                "rviz", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "simulator_environment_map",
                default_value=str(example_dir / "open_room.xml"),
                description="Simulator-only geometry; never grants robot access",
            ),
            DeclareLaunchArgument(
                "semaforr_map_mode",
                default_value="mapless",
                choices=["mapless", "map_enabled"],
            ),
            DeclareLaunchArgument(
                "semaforr_map_path",
                default_value=str(example_dir / "open_room.xml"),
            ),
            DeclareLaunchArgument(
                "map_based_planning",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "map_visualization",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "map_planners",
                default_value="skeleton",
                description=(
                    "Comma-separated planner catalog; use distance and other "
                    "grid planners only with semaforr_map_mode=map_enabled"
                ),
            ),
            OpaqueFunction(function=launch_nodes),
            rviz,
        ]
    )
