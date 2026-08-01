#!/usr/bin/env python3
"""Command-line entrypoint for plotting compute-time summaries from a ROS 2 bag."""

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

# Preserve direct execution from any working directory as well as package imports in tests.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from avt_341_tools.plot_compute_time.bag_reader import (  # noqa: E402
    inspect_bag,
    load_ros_dependencies,
    read_compute_samples,
    validate_bag_path,
)
from avt_341_tools.plot_compute_time.models import (  # noqa: E402
    PlotComputeTimeError,
    default_output_directory,
    format_vehicle_inventory,
    select_compute_topics,
)
from avt_341_tools.plot_compute_time.plotting import render_plots  # noqa: E402


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Plot avt_341_msgs/ComputeTimeArray summaries from ROS 2 SQLite3 or MCAP bags."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "bag",
        type=Path,
        help="ROS 2 bag directory or an individual .mcap/.db3 file",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="Output root; defaults to <bag-name>_compute_times beside the bag",
    )
    parser.add_argument(
        "--vehicle-id",
        dest="vehicle_ids",
        action="append",
        metavar="ID",
        help="Plot only this vehicle ID; repeat to select multiple vehicles",
    )
    parser.add_argument(
        "--list-vehicles",
        action="store_true",
        help="List metadata-discovered compute-time vehicles and exit",
    )
    return parser


def run(arguments: argparse.Namespace) -> int:
    bag_path = arguments.bag.expanduser().resolve()
    validate_bag_path(bag_path)
    dependencies = load_ros_dependencies()
    metadata, available_topics = inspect_bag(bag_path, dependencies)

    if arguments.list_vehicles:
        print(format_vehicle_inventory(metadata, available_topics))
        return 0

    selected_topics = select_compute_topics(available_topics, arguments.vehicle_ids)
    grouped = read_compute_samples(bag_path, selected_topics, dependencies)
    output_root = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else default_output_directory(bag_path)
    )
    try:
        saved_paths = render_plots(grouped, output_root)
    except Exception as exc:
        raise PlotComputeTimeError(
            "Unable to write plots under '{}': {}".format(output_root, exc)
        ) from exc
    if not saved_paths:
        raise PlotComputeTimeError("No plots were generated.")

    print(
        "Saved {} plot{} to {}".format(
            len(saved_paths), "" if len(saved_paths) == 1 else "s", output_root
        )
    )
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    arguments = parser.parse_args(argv)
    try:
        return run(arguments)
    except PlotComputeTimeError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
