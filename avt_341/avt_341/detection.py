"""Detection module"""

from __future__ import annotations

from pathlib import Path
from shutil import copy
from typing import List

from ultralytics import YOLO


class YoloModel:
    """A YOLO model"""

    def __init__(self,
                 base_model_name: str,
                 model_names: Path,
                 working_directory: Path,
                 training_size: int = 640,
                 training_device: str = "cpu",
                 export_size: int = 640,
                 export_device: str = "cpu"):
        """_summary_

        Args:
            base_model_name (str): Name of the base YOLO model to use for warm-starting training.
            working_directory (Path): Path to the working directory
            training_size (int, optional): Image size used for training. Defaults to 640.
            training_device (str, optional): Device used for training. Defaults to "cpu".
            export_size (int, optional): Image size for the exported Torch model. Defaults to 640.
            export_device (str, optional): Device for the exported Torch model. Defaults to "cpu".
        """

        self._base_model_name: str = base_model_name
        """base_model_name (str): Name of the base YOLO model to use for warm-starting training."""
        self._model_names: str = model_names
        self._working_directory: Path = working_directory
        self._base_model_path: Path = Path(self._working_directory,
                                           self._base_model_name)

        self._yolo_wrapper = YOLO(base_model_name)

        self._valid_models: List[str] = [
            'yolov8n', 'yolov8s', 'yolov8m', 'yolov8l', 'yolov8x'
        ]
        self._valid_devices: List[str] = ['cpu', 'cuda']

        self._training_size: int = training_size
        self._validate_device(training_device)
        self._training_device: str = training_device

        self._export_size: int = export_size
        self._validate_device(export_device)
        self._export_device: str = export_device

    def _copy_names_file(self: YoloModel) -> None:

        exported_model: Path = Path(
            self._working_directory,
            f"{self._base_model_path.with_suffix('.torchscript')}")

        copy(
            self._model_names,
            Path(
                exported_model.parent,
                f"{exported_model.stem}-{self._export_device}-{self._export_size}x{self._export_size}.names"
            ))

    def export(self: YoloModel) -> None:
        """Export the loaded model to a TorchScript file."""

        self._yolo_wrapper.export(format="torchscript",
                                  imgsz=self._export_size,
                                  device=self._export_device)

        self._rename_exported_model()
        self._copy_names_file()

    def _rename_exported_model(self) -> None:
        """Rename the TorchScript file exported by the Ultralytics YOLOv8 export
        function to include device type and input tensor size."""

        exported_model: Path = Path(
            self._working_directory,
            f"{self._base_model_path.with_suffix('.torchscript')}")

        exported_model.rename(
            Path(
                exported_model.parent,
                f"{exported_model.stem}-{self._export_device}-{self._export_size}x{self._export_size}{exported_model.suffix}"
            ))

    def _validate_device(self, device: str) -> None:
        """Check whether the supplied device name string is a valid device.

        Args:
            device (str): Device string containing the device type short name.

        Raises:
            InvalidDeviceError: Raised if the provided device string does not match any valid device type.
        """
        if device not in self._valid_devices:
            raise InvalidDeviceError(
                f"Invalid device - valid devices are {self._valid_devices}")
