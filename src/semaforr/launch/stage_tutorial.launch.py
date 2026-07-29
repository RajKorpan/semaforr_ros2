from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("semaforr"))
    config_dir = package_share / "config"
    tutorial_dir = config_dir / "stage_tutorial"

    return LaunchDescription(
        [
            Node(
                package="semaforr",
                executable="semaforr_node",
                name="semaforr",
                output="screen",
                parameters=[
                    str(config_dir / "semaforr.yaml"),
                    {
                        "map.path": str(
                            tutorial_dir / "stage_tutorialS.xml"
                        ),
                        "mission.tasks_path": str(
                            tutorial_dir / "target.conf"
                        ),
                    },
                ],
            )
        ]
    )
