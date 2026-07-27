#!/usr/bin/env python3
"""
Dump every parameter of every running ROS2 node to a single YAML file.

The script discovers all nodes currently visible on the ROS graph, queries each
node's parameter services (``<node>/list_parameters`` and
``<node>/get_parameters``) and writes the resulting key/value pairs to a YAML
file keyed by fully-qualified node name, e.g.::

    /diagnostic_test_node:
      use_sim_time: false
    /mrzr2/avt_341_global_path_node:
      is_empty_waypoints: true
      map_origin_x: 0.0
      map_origin_y: 0.0

Usage:
    python preprocessing/ros_param_dump.py --out_file ../data/ros_params_dump.yaml
"""

import argparse
import re
import sys
import time
from typing import Any, Dict, List, Optional, Pattern

import yaml

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters


NODE_NAME = "ros_param_dump"
DEFAULT_OUT_FILE = "../data/ros_params_dump.yaml"
# Time to let DDS discovery populate the node graph before enumerating.
DEFAULT_DISCOVERY_WAIT_S = 2.0
# Per service wait / call timeout.
DEFAULT_SERVICE_TIMEOUT_S = 5.0
# Nodes whose fully-qualified name matches this regex are skipped entirely.
# - transform_listener_impl: tf2 spins up throw-away hidden listener nodes that
#   are noise here (matched anywhere in the name).
# - /rosbag2_recorder: the recorder node created by `ros2 bag record` (present
#   when this dump runs alongside vehicle_logging); its params aren't user config.
DEFAULT_NODE_IGNORE_REGEX = r"transform_listener_impl|^/rosbag2_recorder$"
# Parameters whose name matches this regex are not fetched. QoS override params
# are per-topic implementation detail rather than user-facing configuration.
DEFAULT_PARAM_IGNORE_REGEX = r"^qos_overrides"
# ``ListParameters`` recurses over all nested parameters when depth is 0.
LIST_PARAMETERS_RECURSIVE_DEPTH = 0


def _write_yaml(data: Dict[str, Dict[str, Any]], out_file: str) -> None:
    with open(out_file, "w") as fh:
        # sort_keys=False preserves the already-sorted insertion order while
        # still emitting block-style, human-readable YAML.
        yaml.safe_dump(data, fh, default_flow_style=False, sort_keys=False)

def _parameter_value_to_python(value: ParameterValue) -> Optional[Any]:
    """Convert an ``rcl_interfaces/ParameterValue`` msg to a native Python value.

    Implemented explicitly (rather than relying on
    ``rclpy.parameter.parameter_value_to_python``) so the script stays portable
    across ROS2 distributions. Array types are normalised to plain ``list`` so
    that PyYAML can serialise them cleanly.
    """
    converters = {
        ParameterType.PARAMETER_BOOL: lambda v: v.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda v: v.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda v: v.double_value,
        ParameterType.PARAMETER_STRING: lambda v: v.string_value,
        ParameterType.PARAMETER_BYTE_ARRAY: lambda v: list(v.byte_array_value),
        ParameterType.PARAMETER_BOOL_ARRAY: lambda v: list(v.bool_array_value),
        ParameterType.PARAMETER_INTEGER_ARRAY: lambda v: [int(x) for x in v.integer_array_value],
        ParameterType.PARAMETER_DOUBLE_ARRAY: lambda v: [float(x) for x in v.double_array_value],
        ParameterType.PARAMETER_STRING_ARRAY: lambda v: list(v.string_array_value),
    }
    convert = converters.get(value.type)
    return convert(value) if convert is not None else None


def _fully_qualified_name(name: str, namespace: str) -> str:
    """Join a node name and namespace into a single fully-qualified name."""
    if namespace.endswith("/"):
        return f"{namespace}{name}"
    return f"{namespace}/{name}"


def _compile_optional(pattern: Optional[str]) -> Optional[Pattern]:
    """Compile ``pattern`` into a regex, or return ``None`` when it is empty.

    An empty/omitted pattern disables the corresponding filter.
    """
    return re.compile(pattern) if pattern else None


class RosParamDumpNode(Node):
    """Utility node that reads every other node's parameters off the graph."""

    def __init__(
        self,
        service_timeout_s: float,
        node_ignore: Optional[Pattern] = None,
        param_ignore: Optional[Pattern] = None,
    ) -> None:
        super().__init__(NODE_NAME)
        self._service_timeout_s = service_timeout_s
        self._node_ignore = node_ignore
        self._param_ignore = param_ignore

    def _should_dump_node(self, name: str, node_fqn: str, self_fqn: str) -> bool:
        """Decide whether ``node_fqn`` should be included in the dump."""
        if name.startswith("_"):
            return False
        if node_fqn == self_fqn:
            return False
        if self._node_ignore is not None and self._node_ignore.search(node_fqn):
            self.get_logger().info(f"Skipping node (matched node ignore): {node_fqn}")
            return False
        return True

    def _is_param_ignored(self, name: str) -> bool:
        """Return True if ``name`` matches the parameter-ignore regex."""
        return self._param_ignore is not None and self._param_ignore.search(name) is not None

    def discover_node_names(self, discovery_wait_s: float) -> List[str]:
        """Spin briefly to let discovery settle, then return node FQNs.

        Excludes the dumper node itself, hidden nodes (names starting with an
        underscore) and any node matching the node-ignore regex.
        """
        self.get_logger().info(f"Waiting {discovery_wait_s:.1f}s for node discovery...")
        deadline = time.time() + discovery_wait_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        self_fqn = self.get_fully_qualified_name()
        discovered: List[str] = []
        for name, namespace in self.get_node_names_and_namespaces():
            node_fqn = _fully_qualified_name(name, namespace)
            if self._should_dump_node(name, node_fqn, self_fqn):
                discovered.append(node_fqn)

        discovered = sorted(set(discovered))
        self.get_logger().info(f"Discovered {len(discovered)} node(s) to dump.")
        return discovered

    def _spin_for_result(self, future: Future) -> Optional[Any]:
        """Spin until ``future`` completes or the service timeout elapses."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout_s)
        if not future.done():
            return None
        if future.exception() is not None:
            return None
        return future.result()

    def _call_service(self, client, request) -> Optional[Any]:
        """Wait for ``client``'s service then call it, returning the response."""
        if not client.wait_for_service(timeout_sec=self._service_timeout_s):
            return None
        return self._spin_for_result(client.call_async(request))

    def list_parameters(self, node_fqn: str) -> List[str]:
        """Return the parameter names exposed by ``node_fqn`` (empty on failure)."""
        client = self.create_client(ListParameters, f"{node_fqn}/list_parameters")
        try:
            request = ListParameters.Request()
            request.depth = LIST_PARAMETERS_RECURSIVE_DEPTH
            response = self._call_service(client, request)
            if response is None:
                self.get_logger().warning(f"Could not list parameters for {node_fqn}")
                return []
            return list(response.result.names)
        finally:
            self.destroy_client(client)

    def get_parameters(self, node_fqn: str, names: List[str]) -> Dict[str, Any]:
        """Return {param name: value} for ``names`` on ``node_fqn``."""
        if not names:
            return {}
        client = self.create_client(GetParameters, f"{node_fqn}/get_parameters")
        try:
            request = GetParameters.Request()
            request.names = names
            response = self._call_service(client, request)
            if response is None:
                self.get_logger().warning(f"Could not get parameters for {node_fqn}")
                return {}
            return {
                name: _parameter_value_to_python(value)
                for name, value in zip(names, response.values)
            }
        finally:
            self.destroy_client(client)

    def get_param_dict(self, discovery_wait_s: float) -> Dict[str, Dict[str, Any]]:
        """Discover every node and collect its parameters into a nested dict."""
        params_dict: Dict[str, Dict[str, Any]] = {}
        for node_fqn in self.discover_node_names(discovery_wait_s):
            self.get_logger().info(f"Reading parameters from {node_fqn}")
            names = sorted(
                name for name in self.list_parameters(node_fqn)
                if not self._is_param_ignored(name)
            )
            params_dict[node_fqn] = self.get_parameters(node_fqn, names)
        return params_dict

    def dump(self, discovery_wait_s: float, out_file: str):
        params_dict = self.get_param_dict(discovery_wait_s)
        if params_dict:
            _write_yaml(params_dict, out_file)
            total_params = sum(len(params) for params in params_dict.values())
            self.get_logger().info(f"[DONE] Wrote {total_params} parameter(s) from {len(params_dict)} node(s) to {out_file}")
        else:
            self.get_logger().warn("[DONE] No nodes found to dump parameters for.")


def parse_arguments(
    argv: Optional[List[str]] = None,
    prefix: str = "",
    include_out_file: bool = True,
) -> argparse.Namespace:
    """Parse the param-dump options.

    ``prefix`` is prepended to every optional's flag (but not to its ``dest``),
    so a caller like ``vehicle_logging.py`` can pass ``prefix="param_dump_"`` to
    expose ``--param_dump_discovery_wait`` etc. on its own command line while
    downstream code still reads the stable ``args.discovery_wait`` attribute.

    ``include_out_file`` can be set False when the caller supplies the output
    path itself (see :func:`dump_ros_params_from_argv`) so ``--out_file`` is not
    a valid flag in that context.
    """
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    if include_out_file:
        parser.add_argument(
            "--out_file",
            type=str,
            default=DEFAULT_OUT_FILE,
            help="Path to the output YAML file.",
        )
    parser.add_argument(
        f"--{prefix}discovery_wait",
        dest="discovery_wait",
        type=float,
        default=DEFAULT_DISCOVERY_WAIT_S,
        help="Seconds to wait for node discovery before enumerating nodes.",
    )
    parser.add_argument(
        f"--{prefix}service_timeout",
        dest="service_timeout",
        type=float,
        default=DEFAULT_SERVICE_TIMEOUT_S,
        help="Per-node timeout (seconds) for the parameter service calls.",
    )
    parser.add_argument(
        f"--{prefix}ignore_nodes",
        dest="ignore_nodes",
        type=str,
        default=DEFAULT_NODE_IGNORE_REGEX,
        help="Regex; nodes whose fully-qualified name matches are skipped "
             "entirely. Pass an empty string to disable.",
    )
    parser.add_argument(
        f"--{prefix}ignore_params",
        dest="ignore_params",
        type=str,
        default=DEFAULT_PARAM_IGNORE_REGEX,
        help="Regex; parameters whose name matches are not fetched. "
             "Pass an empty string to disable.",
    )
    return parser.parse_args(argv)


def dump_ros_params(
    out_file: str,
    discovery_wait_s: float = DEFAULT_DISCOVERY_WAIT_S,
    service_timeout_s: float = DEFAULT_SERVICE_TIMEOUT_S,
    ignore_nodes: str = DEFAULT_NODE_IGNORE_REGEX,
    ignore_params: str = DEFAULT_PARAM_IGNORE_REGEX,
) -> int:
    """Discover every running node, collect its parameters and write them to
    ``out_file``.

    This is the shared core used by both the standalone CLI (:func:`main`) and
    external callers such as ``vehicle_logging.py`` (via
    :func:`dump_ros_params_from_argv`).
    """
    rclpy.init(args=sys.argv)
    try:
        node = RosParamDumpNode(
            service_timeout_s=service_timeout_s,
            node_ignore=_compile_optional(ignore_nodes),
            param_ignore=_compile_optional(ignore_params),
        )
        try:
            node.dump(discovery_wait_s, out_file)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

    return 0


def dump_ros_params_from_argv(
    out_file: str,
    argv: Optional[List[str]] = None,
    prefix: str = "param_dump_",
) -> int:
    """Entry point for callers that supply ``out_file`` themselves rather than
    via ``--out_file`` (e.g. ``vehicle_logging.py`` points it at the rosbag
    output directory).

    The remaining tunables are parsed from ``argv`` under ``prefix`` so that,
    on the caller's command line, flags like ``--param_dump_discovery_wait``
    clearly belong to this sub-module.
    """
    args = parse_arguments(argv=argv, prefix=prefix, include_out_file=False)
    return dump_ros_params(
        out_file=out_file,
        discovery_wait_s=args.discovery_wait,
        service_timeout_s=args.service_timeout,
        ignore_nodes=args.ignore_nodes,
        ignore_params=args.ignore_params,
    )


def main() -> int:
    args = parse_arguments()
    return dump_ros_params(
        out_file=args.out_file,
        discovery_wait_s=args.discovery_wait,
        service_timeout_s=args.service_timeout,
        ignore_nodes=args.ignore_nodes,
        ignore_params=args.ignore_params,
    )


if __name__ == "__main__":
    raise SystemExit(main())
