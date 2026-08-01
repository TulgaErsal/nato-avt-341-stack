"""ROS 2 metadata discovery and ComputeTimeArray bag deserialization."""

import math
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from .models import (
    COMPUTE_TIME_ARRAY_TYPE,
    COMPUTE_TIME_TOPIC_SUFFIX,
    MILLISECONDS_PER_SECOND,
    ComputeSample,
    ComputeTopic,
    GroupedSamples,
    PlotComputeTimeError,
    RosDependencies,
    parse_section_path,
    vehicle_id_from_topic,
)


def load_ros_dependencies() -> RosDependencies:
    """Load native ROS dependencies and turn loader failures into actionable errors."""

    try:
        import rosbag2_py
        from avt_341_msgs.msg import ComputeTimeArray
        from rclpy.serialization import deserialize_message
    except (ImportError, OSError) as exc:
        raise PlotComputeTimeError(
            "Unable to load the ROS 2 bag/message Python libraries. Run this utility "
            "from a sourced ROS 2 workspace containing avt_341_msgs, and ensure the "
            "rosbag2 SQLite3/MCAP storage plugins and their native libraries are installed. "
            "Original error: {}".format(exc)
        ) from exc

    return RosDependencies(
        rosbag2_py=rosbag2_py,
        deserialize_message=deserialize_message,
        compute_time_array_type=ComputeTimeArray,
    )


def _metadata_topic_fields(topic_information: Any) -> Tuple[str, str, int]:
    """Extract stable fields from Humble's TopicInformation Python binding."""

    topic_metadata = topic_information.topic_metadata
    return (
        str(topic_metadata.name),
        str(topic_metadata.type),
        int(topic_information.message_count),
    )


def discover_compute_topics(metadata: Any) -> Tuple[List[ComputeTopic], List[Tuple[str, str]]]:
    """Find valid compute-time topics and same-name topics with an unexpected type."""

    topics: List[ComputeTopic] = []
    wrong_types: List[Tuple[str, str]] = []

    for topic_information in metadata.topics_with_message_count:
        topic_name, topic_type, message_count = _metadata_topic_fields(topic_information)
        vehicle_id = vehicle_id_from_topic(topic_name)
        if vehicle_id is None:
            continue
        if topic_type != COMPUTE_TIME_ARRAY_TYPE:
            wrong_types.append((topic_name, topic_type))
            continue
        topics.append(
            ComputeTopic(
                vehicle_id=vehicle_id,
                topic_name=topic_name,
                message_count=message_count,
            )
        )

    topics.sort(key=lambda topic: (topic.vehicle_id, topic.topic_name))
    return topics, wrong_types


def validate_bag_path(bag_path: Path) -> None:
    """Validate the supported input shapes before loading native ROS libraries."""

    if not bag_path.exists():
        raise PlotComputeTimeError("Bag path does not exist: {}".format(bag_path))
    if bag_path.is_file() and bag_path.suffix.lower() not in (".mcap", ".db3"):
        raise PlotComputeTimeError(
            "Unsupported bag file '{}'; expected a .mcap file, a .db3 file, "
            "or a ROS 2 bag directory.".format(bag_path)
        )


def inspect_bag(bag_path: Path, dependencies: RosDependencies) -> Tuple[Any, List[ComputeTopic]]:
    """Read rosbag metadata and return its compute-time topic inventory."""

    validate_bag_path(bag_path)
    try:
        metadata = dependencies.rosbag2_py.Info().read_metadata(str(bag_path), "")
    except Exception as exc:
        raise PlotComputeTimeError(
            "Unable to read ROS 2 metadata from '{}': {}".format(bag_path, exc)
        ) from exc

    topics, wrong_types = discover_compute_topics(metadata)
    if not topics:
        if wrong_types:
            details = ", ".join(
                "{} ({})".format(topic_name, topic_type)
                for topic_name, topic_type in wrong_types
            )
            raise PlotComputeTimeError(
                "Compute-time topic names were found, but none has type {}: {}".format(
                    COMPUTE_TIME_ARRAY_TYPE, details
                )
            )
        raise PlotComputeTimeError(
            "No '<vehicle_id>{}' topics of type {} were found in the bag metadata.".format(
                COMPUTE_TIME_TOPIC_SUFFIX, COMPUTE_TIME_ARRAY_TYPE
            )
        )

    duplicate_ids: Dict[str, List[str]] = {}
    for topic in topics:
        duplicate_ids.setdefault(topic.vehicle_id, []).append(topic.topic_name)
    ambiguous = {
        vehicle_id: names for vehicle_id, names in duplicate_ids.items() if len(names) > 1
    }
    if ambiguous:
        details = "; ".join(
            "{}: {}".format(vehicle_id, ", ".join(names))
            for vehicle_id, names in sorted(ambiguous.items())
        )
        raise PlotComputeTimeError(
            "Multiple compute-time topics map to the same vehicle ID: {}".format(details)
        )

    return metadata, topics


def _default_warning(message: str) -> None:
    print("warning: {}".format(message), file=sys.stderr)


def read_compute_samples(
    bag_path: Path,
    selected_topics: Sequence[ComputeTopic],
    dependencies: RosDependencies,
    warning: Callable[[str], None] = _default_warning,
) -> GroupedSamples:
    """Read, deserialize, validate, and group selected compute-time messages."""

    rosbag2_py = dependencies.rosbag2_py
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
            rosbag2_py.ConverterOptions(
                input_serialization_format="", output_serialization_format=""
            ),
        )
        selected_topic_names = [topic.topic_name for topic in selected_topics]
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected_topic_names))
    except Exception as exc:
        raise PlotComputeTimeError(
            "Unable to open '{}' with rosbag2 SequentialReader: {}".format(bag_path, exc)
        ) from exc

    vehicle_by_topic = {topic.topic_name: topic.vehicle_id for topic in selected_topics}
    grouped: GroupedSamples = {}
    origin_timestamp_ns: Optional[int] = None
    messages_seen = {topic_name: 0 for topic_name in vehicle_by_topic}
    valid_samples_seen = {topic_name: 0 for topic_name in vehicle_by_topic}
    groups_seen: Dict[Tuple[str, str], int] = {}

    while reader.has_next():
        try:
            topic_name, serialized_data, bag_timestamp_ns = reader.read_next()
        except Exception as exc:
            raise PlotComputeTimeError(
                "Failed while reading the next message from '{}': {}".format(bag_path, exc)
            ) from exc

        if topic_name not in vehicle_by_topic:
            continue
        messages_seen[topic_name] += 1
        if origin_timestamp_ns is None:
            origin_timestamp_ns = int(bag_timestamp_ns)

        try:
            message = dependencies.deserialize_message(
                serialized_data, dependencies.compute_time_array_type
            )
        except Exception as exc:
            raise PlotComputeTimeError(
                "Unable to deserialize {} at bag timestamp {} as {}: {}".format(
                    topic_name, bag_timestamp_ns, COMPUTE_TIME_ARRAY_TYPE, exc
                )
            ) from exc

        vehicle_id = vehicle_by_topic[topic_name]
        tag = str(message.tag)
        group_key = (vehicle_id, tag)
        groups_seen.setdefault(group_key, 0)
        relative_time_seconds = (int(bag_timestamp_ns) - origin_timestamp_ns) * 1e-9

        for compute_time in message.compute_times:
            section_id = str(compute_time.section_id)
            try:
                parse_section_path(section_id)
            except ValueError as exc:
                warning(
                    "Skipping malformed section_id {!r} on {} at {}: {}".format(
                        section_id, topic_name, bag_timestamp_ns, exc
                    )
                )
                continue

            mean_seconds = float(compute_time.time)
            std_seconds = float(compute_time.time_std)
            if (
                not math.isfinite(mean_seconds)
                or not math.isfinite(std_seconds)
                or mean_seconds < 0.0
                or std_seconds < 0.0
            ):
                warning(
                    "Skipping invalid statistics for section {!r} on {} at {}: "
                    "mean={}, std={}".format(
                        section_id,
                        topic_name,
                        bag_timestamp_ns,
                        mean_seconds,
                        std_seconds,
                    )
                )
                continue

            sample = ComputeSample(
                relative_time_seconds=relative_time_seconds,
                mean_milliseconds=mean_seconds * MILLISECONDS_PER_SECOND,
                std_milliseconds=std_seconds * MILLISECONDS_PER_SECOND,
                synthesized=bool(compute_time.auto_parent_stats),
            )
            grouped.setdefault(vehicle_id, {}).setdefault(tag, {}).setdefault(
                section_id, []
            ).append(sample)
            valid_samples_seen[topic_name] += 1
            groups_seen[group_key] += 1

    if origin_timestamp_ns is None:
        raise PlotComputeTimeError(
            "The selected compute-time topics contain no messages: {}".format(
                ", ".join(vehicle_by_topic)
            )
        )
    if not grouped:
        raise PlotComputeTimeError(
            "The selected compute-time messages contain no valid section samples."
        )

    for topic_name, count in messages_seen.items():
        if count == 0:
            warning("Selected topic {} contains no messages.".format(topic_name))
        elif valid_samples_seen[topic_name] == 0:
            warning("Selected topic {} contains no valid section samples.".format(topic_name))
    for (vehicle_id, tag), sample_count in groups_seen.items():
        if sample_count == 0:
            warning(
                "Vehicle {!r}, tag {!r} contains no valid section samples.".format(
                    vehicle_id, tag
                )
            )
    return grouped
