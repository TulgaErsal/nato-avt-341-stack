"""Nodes module"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from avt_341.detection import YoloModel


class AvtNode(Node):

    def __init__(self, name: str) -> None:

        super().__init__(name)


class YoloGym(AvtNode):

    def __init__(self: YoloGym) -> None:

        super().__init__('yolo_gym')

        self._training_size: Optional[int] = None
        self._training_device: Optional[str] = None
        self._export_size: Optional[int] = None
        self._export_device: Optional[str] = None
        self._base_model_path: Optional[Path] = None

        self._get_parameters()
        self._initialize()
        self._create_timers()

    def _get_parameters(self: YoloGym) -> None:

        self.declare_parameter("model.base", "yolov8m")
        self._base_model = self.get_parameter(
            "model.base").get_parameter_value().string_value
        self._base_model_path = self._get_model(self._base_model)

        self.declare_parameter("model.names", "")
        self._model_names = self.get_parameter(
            "model.names").get_parameter_value().string_value

        self.declare_parameter("training.size", 640)
        self._training_size = self.get_parameter(
            "training.size").get_parameter_value().integer_value

        self.declare_parameter("training.device", "cpu")
        self._training_device = self.get_parameter(
            "training.device").get_parameter_value().string_value

        self.declare_parameter("export.size", 640)
        self._export_size = self.get_parameter(
            "export.size").get_parameter_value().integer_value

        self.declare_parameter("export.device", "cuda")
        self._export_device = self.get_parameter(
            "export.device").get_parameter_value().string_value

    def _initialize(self: YoloGym) -> None:

        self._yolo_model = YoloModel(str(self._base_model_path),
                                     self._model_names,
                                     self._base_model_path.parent,
                                     training_size=self._training_size,
                                     training_device=self._training_device,
                                     export_size=self._export_size,
                                     export_device=self._export_device)

    def _create_timers(self: YoloGym) -> None:

        self._oneshot_timer = self.create_timer(1.0,
                                                self._oneshot_timer_callback)

    def _get_model(self: YoloGym, model: str) -> Path:

        return Path(get_package_share_directory('avt_341')) / Path(
            f"ml_weights/detection/models/{model}.pt")

    def _oneshot_timer_callback(self: YoloGym) -> None:

        # Cancel the timer after the first callback.
        self._oneshot_timer.cancel()

        self._yolo_model.export()
