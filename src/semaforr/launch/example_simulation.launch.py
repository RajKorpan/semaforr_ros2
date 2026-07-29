"""Run the installed deterministic SemaFORR tutorial with optional RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    semaforr = Node(
        package="semaforr",
        executable="semaforr_node",
        name="semaforr",
        output="screen",
        parameters=[
            str(config_dir / "semaforr.yaml"),
            {
                "map.path": str(example_dir / "open_room.xml"),
                "map.length_m": 12,
                "map.height_m": 12,
                "mission.tasks_path": str(example_dir / "mission.conf"),
            },
        ],
    )

    simulator = Node(
        package="semaforr",
        executable="semaforr_record_baseline",
        name="semaforr_example_simulator",
        output="screen",
        arguments=[
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
        ],
    )
    delayed_simulator = TimerAction(
        period=startup_delay,
        actions=[simulator],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="semaforr_rviz",
        output="screen",
        arguments=["-d", str(package_share / "rviz" / "semaforr.rviz")],
        condition=IfCondition(use_rviz),
    )

    stop_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=simulator,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="The deterministic SemaFORR example completed"
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value=str(default_output),
                description="Destination for the structured decision trace",
            ),
            DeclareLaunchArgument(
                "duration",
                default_value="20.0",
                description="Deterministic scenario duration in seconds",
            ),
            DeclareLaunchArgument(
                "sensor_cutoff",
                default_value="-1.0",
                description=(
                    "Stop sensors at this scenario time; negative disables it"
                ),
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="3.0",
                description=(
                    "Delay fixture playback while navigation initializes"
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                choices=["true", "false"],
                description="Start RViz with the installed SemaFORR layout",
            ),
            semaforr,
            delayed_simulator,
            rviz,
            stop_when_complete,
        ]
    )
