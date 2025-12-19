#!/usr/bin/env python3

import logging

import os
import shlex
import signal
import subprocess

from pathlib import Path
from time import time
from typing import List, Optional, Union


class Launcher:

    def __init__(self) -> None:

        self._configure_logger()

        logging.info("Initializing ROS Docker entrypoint script ...")

    def launch(self) -> None:

        logging.info("Launching ROS container ...")

        self._get_paths()
        self._source_global_environment()
        self._log_environment_variables()
        self._optional_build()
        self._source_local_environment()
        self._parse()

    def _configure_logger(self) -> None:

        logging.getLogger().setLevel(logging.INFO)
        logging.getLogger().name = "ROS entrypoint"
        formatter = logging.Formatter("[%(levelname)7s] :: %(name)s :: %(message)s")
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        logging.getLogger().addHandler(handler)

    def _log_environment_variables(self) -> None:

        keys: List[str] = [
            f"{key} = {value}"
            for (key, value) in dict(os.environ).items()
            if key.startswith("DOCKER_ROS")
        ]

        if keys != []:
            [logging.info('Found environment variable: "%s"', key) for key in keys]
        else:
            logging.warning("No ROS Docker container environment keys specified!")

    def _get_environment_variable(self, key: str) -> str:

        value = os.environ.get(key)
        if value is not None:
            return value
        raise ValueError('Missing environment variable "%s"!', key)

    def _get_positional_argument(
        self, function: callable, prefix: str = ""
    ) -> Union[str, List[str]]:

        try:
            value = function()
        except:
            logging.error("ERROR: %s", prefix)
            return ""

        if value is None:
            return ""

        return (
            f"{prefix} {value}"
            if type(value) == str
            else " ".join([f"{prefix} {entry}" for entry in value])
        )

    def _get_paths(self) -> None:

        try:
            self._ros_distribution = self._get_environment_variable(
                "DOCKER_ROS_DISTRIBUTION"
            )
            logging.info('Setting ROS distribution to "%s"', self._ros_distribution)
        except:
            self._ros_distribution = "humble"
            logging.info('No ROS distribution specified, defaulting to "Humble"')

        try:
            self._ros_workspace = Path(
                self._get_environment_variable("DOCKER_ROS_WORKSPACE")
            )
            logging.info('Setting ROS workspace to "%s"', self._ros_workspace)
        except:
            logging.info("No ROS workspace specified, defaulting to ~/ROS ...")
            self._ros_workspace = Path.home() / Path("ROS")

    def _source_file(self, path: str) -> None:

        if not Path(path).exists():
            logging.error(
                'Could not find script to be sourced at "%s"!', Path(path).as_posix()
            )
            raise RuntimeError("Invalid source file!")

        return f"/bin/bash -c 'source {Path(path).as_posix()}"

    def _source_local_environment(self) -> None:

        logging.info("Sourcing environment script from the local ROS workspace ...")

        return f"source {self._ros_workspace / Path('install/local_setup.bash').as_posix()}"

    def _source_global_environment(self) -> None:

        logging.info("Sourcing environment script from the global ROS distribution ...")

        return f"source /opt/ros/{self._ros_distribution}/setup.bash"

    def _optional_build(self) -> None:

        if os.environ.get("DOCKER_ROS_BUILD"):
            self._run_build()

    def _get_cmake_args(self) -> Optional[List[str]]:

        try:
            cmake_args: List[str] = self._get_environment_variable(
                "DOCKER_ROS_CMAKE_ARGS"
            )
            logging.info(
                "Found CMake arguments to be forwarded to colcon: %s",
                [arg for arg in cmake_args],
            )

            return cmake_args
        except:
            logging.info("No CMake arguments specified ...")

            raise ValueError

    def _get_params_file(self) -> None:

        if os.environ.get("DOCKER_ROS_PARAMETERS_FILE"):
            ros_params: List[str] = os.environ["DOCKER_ROS_PARAMETERS_FILE"]
            logging.info(
                "Found ROS parameters file to be loaded: %s",
                ros_params,
            )

            return ros_params

        logging.info("No ROS parameters file specified ...")
        return None

    def _run_build(self) -> None:

        logging.info("Building packages using colcon ...")

        try:
            cmake_args_cmd: str = f"--cmake_args {self._get_cmake_args()}"
        except:
            cmake_args_cmd = ""

        self._process = subprocess.run(
            shlex.split(
                f'/bin/bash -c "cd {self._ros_workspace} && colcon build {cmake_args_cmd}"'
            )
        )

        self._source_local_environment()

    def _parse(self) -> None:

        action = self._get_environment_variable("DOCKER_ROS_ACTION").lower()

        logging.info('Selected action: "%s"', action)

        if action == "run":
            self._run_node_run()
        elif action == "launch":
            self._run_node_launch()
        elif action == "play":
            self._run_bag_play()
        elif action == "record":
            self._run_bag_record()
        elif action == "shell":
            self._run_shell()
        else:
            raise ValueError("Invalid action!")

    def _run_node_run(self) -> None:

        logging.info("Running ROS node ...")

        package = self._get_environment_variable("DOCKER_ROS_PACKAGE")
        executable = self._get_environment_variable("DOCKER_ROS_EXECUTABLE")

        logging.info("Package: %s", {package})
        logging.info("Executable: %s", {executable})

        parameters_file = self._get_positional_argument(
            self._get_params_file, "--params-file"
        )
        remappings = self._get_positional_argument(self._get_remappings, "-r")
        parameters = self._get_positional_argument(self._get_parameters, "-p")

        ros_args_command = (
            "--ros-args"
            if any((parameters_file != "", parameters != "", remappings != ""))
            else ""
        )

        try:
            self._get_environment_variable("DOCKER_ROS_DEBUG")
            prefix = "--prefix 'gdb -ex run --args'"
        except:
            prefix = ""

        command = f"/bin/bash -c '{self._source_global_environment()} && {self._source_local_environment()} && ros2 run {prefix} {package} {executable} {ros_args_command} {parameters_file} {parameters} {remappings}'"

        logging.info('Executing command "%s"', command)

        self._process = subprocess.run(shlex.split(command))

    def _get_parameters(self) -> None:

        try:
            parameters = self._get_environment_variable("DOCKER_ROS_PARAMETERS").split(
                ","
            )
            [
                logging.info('Found parameter "%s"', parameter)
                for parameter in parameters
            ]
            return parameters
        except:
            return []

    def _get_remappings(self) -> None:

        try:
            remappings = self._get_environment_variable("DOCKER_ROS_REMAPPINGS").split(
                ","
            )
            [
                logging.info('Found remapping "%s"', remapping)
                for remapping in remappings
            ]
            return remappings
        except:
            return []

    def _run_node_launch(self) -> None:

        raise NotImplemented("")

    def _run_bag_play(self) -> None:

        remappings = self._get_remappings()
        if remappings != []:
            remappings = f"--remap {' '.join(remappings)}"

        command = f"/bin/bash -c '{self._source_global_environment()} && {self._source_local_environment()} && ros2 bag play {remappings} --clock 1000 --loop {self._get_bag_path()}'"
        
        logging.info('Executing command "%s"', command)

        signal.signal(signal.SIGINT, self._exit_gracefully)

        self._process = subprocess.Popen(
            shlex.split(command),
            stdin=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        self._process.wait()

    def _get_bag_path(self) -> str:

        try:
            rosbag_path = self._get_environment_variable("DOCKER_ROS_BAG_PATH")
        except:
            rosbag_path = Path.home() / Path("bags")
        return rosbag_path

    def _run_bag_record(self) -> None:

        try:
            rosbag_name = self._get_environment_variable("DOCKER_ROS_BAG_NAME")
        except:
            rosbag_name = "bag"

        command = f"/bin/bash -c '{self._source_global_environment()} && {self._source_local_environment()} && ros2 bag record --all -s mcap -o {self._get_bag_path()}/{str(int(time()))}-{rosbag_name}'"
        signal.signal(signal.SIGINT, self._exit_gracefully)

        self._process = subprocess.Popen(
            shlex.split(command),
            stdin=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        self._process.wait()

    def _exit_gracefully(self, a, b):
        logging.warning("Processing SIGTERM signal ...")
        os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)

    def _run_shell(self) -> None:

        self._process = subprocess.run(shlex.split("/bin/bash -l"), shell=True)


def main():
    Launcher().launch()


if __name__ == "__main__":
    main()
