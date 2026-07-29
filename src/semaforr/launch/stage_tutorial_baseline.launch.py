from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("semaforr"))
    config_dir = package_share / "config"
    tutorial_dir = config_dir / "stage_tutorial"

    output = LaunchConfiguration("output")
    duration = LaunchConfiguration("duration")
    sensor_cutoff = LaunchConfiguration("sensor_cutoff")

    semaforr = Node(
        package="semaforr",
        executable="semaforr_node",
        name="semaforr",
        output="screen",
        parameters=[
            str(config_dir / "semaforr.yaml"),
            {
                "map.path": str(tutorial_dir / "stage_tutorialS.xml"),
                "mission.tasks_path": str(tutorial_dir / "target.conf"),
            }
        ],
    )

    recorder = Node(
        package="semaforr",
        executable="semaforr_record_baseline",
        name="semaforr_baseline_recorder",
        output="screen",
        arguments=[
            "--output",
            output,
            "--duration",
            duration,
            "--sensor-cutoff",
            sensor_cutoff,
        ],
    )

    stop_after_recording = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="The SemaFORR baseline recording completed"
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value="baseline-results/stage_tutorial.actual.json",
                description="Destination for the runtime characterization trace",
            ),
            DeclareLaunchArgument(
                "duration",
                default_value="20.0",
                description="Scenario duration in seconds",
            ),
            DeclareLaunchArgument(
                "sensor_cutoff",
                default_value="-1.0",
                description=(
                    "Stop publishing sensors at this time; negative disables "
                    "the cutoff"
                ),
            ),
            semaforr,
            recorder,
            stop_after_recording,
        ]
    )
