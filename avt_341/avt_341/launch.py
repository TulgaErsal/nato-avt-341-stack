from __future__ import annotations

from launch.events import Shutdown
from launch_ros.actions import Node


class AvtNodeLaunchDescription(Node):
    """Launch description wrapper class for AVT-341 autonomy stack nodes."""

    def __init__(self, *args, **kwargs):
        """_summary_
        """

        super().__init__(
            namespace="avt_341",
            on_exit=Shutdown(),
            emulate_tty=True,
            output="both",
            *args,
            **kwargs,
        )


class YoloGymNodeLaunchDescription(AvtNodeLaunchDescription):
    """Launch description wrapper for the YOLO gym node."""

    def __init__(self, *args, **kwargs):
        """_summary_
        """

        super().__init__(*args, **kwargs)
