"""Shared data models, hierarchy helpers, and output path handling."""

import hashlib
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple


COMPUTE_TIME_TOPIC_SUFFIX = "/avt_341/compute_times"
COMPUTE_TIME_ARRAY_TYPE = "avt_341_msgs/msg/ComputeTimeArray"
MILLISECONDS_PER_SECOND = 1000.0


class PlotComputeTimeError(RuntimeError):
    """An expected, user-facing failure while processing a bag."""


@dataclass(frozen=True)
class ComputeTopic:
    """A compute-time topic discovered in rosbag metadata."""

    vehicle_id: str
    topic_name: str
    message_count: int


@dataclass(frozen=True)
class ComputeSample:
    """One published running-statistics sample, converted to milliseconds."""

    relative_time_seconds: float
    mean_milliseconds: float
    std_milliseconds: float
    synthesized: bool


@dataclass(frozen=True)
class RosDependencies:
    """ROS imports kept together so pure helpers remain independently testable."""

    rosbag2_py: Any
    deserialize_message: Callable[[bytes, Any], Any]
    compute_time_array_type: Any


# vehicle id -> tag -> section id -> samples
GroupedSamples = Dict[str, Dict[str, Dict[str, List[ComputeSample]]]]


def vehicle_id_from_topic(topic_name: str) -> Optional[str]:
    """Return the prefix before the compute-time topic suffix, without outer slashes."""

    normalized = "/" + topic_name.strip("/")
    if not normalized.endswith(COMPUTE_TIME_TOPIC_SUFFIX):
        return None

    vehicle_id = normalized[: -len(COMPUTE_TIME_TOPIC_SUFFIX)].strip("/")
    return vehicle_id or None


def select_compute_topics(
    available_topics: Sequence[ComputeTopic], requested_vehicle_ids: Optional[Sequence[str]]
) -> List[ComputeTopic]:
    """Select requested vehicles, or every discovered vehicle when no filter was given."""

    if not requested_vehicle_ids:
        return list(available_topics)

    requested = list(dict.fromkeys(requested_vehicle_ids))
    by_vehicle = {topic.vehicle_id: topic for topic in available_topics}
    unknown = [vehicle_id for vehicle_id in requested if vehicle_id not in by_vehicle]
    if unknown:
        raise PlotComputeTimeError(
            "Unknown vehicle ID(s): {}. Available vehicle IDs: {}".format(
                ", ".join(unknown), ", ".join(sorted(by_vehicle))
            )
        )
    return [by_vehicle[vehicle_id] for vehicle_id in requested]


def format_vehicle_inventory(metadata: Any, topics: Sequence[ComputeTopic]) -> str:
    """Format metadata-backed vehicle choices for --list-vehicles."""

    storage_identifier = str(getattr(metadata, "storage_identifier", "unknown"))
    lines = ["Storage format: {}".format(storage_identifier), "Compute-time vehicles:"]
    for topic in topics:
        lines.append(
            "  {}  topic={}  messages={}".format(
                topic.vehicle_id, topic.topic_name, topic.message_count
            )
        )
    return "\n".join(lines)


def parse_section_path(section_id: str) -> Tuple[str, ...]:
    """Parse a recorder section ID and reject ambiguous hierarchy forms."""

    if not section_id:
        raise ValueError("section_id is empty")
    parts = tuple(section_id.split("/"))
    if any(not part for part in parts):
        raise ValueError("section_id contains an empty hierarchy component")
    if any(part in (".", "..") for part in parts):
        raise ValueError("section_id contains '.' or '..' as a hierarchy component")
    return parts


def direct_children(parent_id: str, section_ids: Iterable[str]) -> List[str]:
    """Return only sections exactly one hierarchy level below parent_id."""

    parent_path = parse_section_path(parent_id)
    children: List[str] = []
    for section_id in section_ids:
        if section_id == parent_id:
            continue
        section_path = parse_section_path(section_id)
        if len(section_path) == len(parent_path) + 1 and section_path[:-1] == parent_path:
            children.append(section_id)
    return sorted(children, key=parse_section_path)


_UNSAFE_PATH_CHARACTERS = re.compile(r'[<>:"|?*\x00-\x1f]')
_WINDOWS_RESERVED_NAMES = {
    "CON",
    "PRN",
    "AUX",
    "NUL",
    "COM1",
    "COM2",
    "COM3",
    "COM4",
    "COM5",
    "COM6",
    "COM7",
    "COM8",
    "COM9",
    "LPT1",
    "LPT2",
    "LPT3",
    "LPT4",
    "LPT5",
    "LPT6",
    "LPT7",
    "LPT8",
    "LPT9",
}


def _short_hash(value: str) -> str:
    return hashlib.sha1(value.encode("utf-8")).hexdigest()[:8]


def safe_path_component(value: str, fallback: str) -> str:
    """Create a readable cross-platform path component from a ROS identifier."""

    safe = value.strip().replace("/", "__").replace("\\", "__")
    safe = _UNSAFE_PATH_CHARACTERS.sub("_", safe)
    safe = re.sub(r"\s+", "_", safe).strip(" ._")
    if not safe:
        safe = fallback
    if safe.split(".", 1)[0].upper() in _WINDOWS_RESERVED_NAMES:
        safe = "_" + safe
    if len(safe) > 140:
        safe = "{}__{}".format(safe[:130].rstrip(" ._"), _short_hash(value))
    return safe


def unique_safe_names(values: Iterable[str], fallback: str) -> Dict[str, str]:
    """Map identifiers to safe names, hashing only identifiers that otherwise collide."""

    unique_values = sorted(set(values))
    values_by_base: Dict[str, List[str]] = {}
    for value in unique_values:
        values_by_base.setdefault(safe_path_component(value, fallback), []).append(value)

    result: Dict[str, str] = {}
    for base, originals in values_by_base.items():
        if len(originals) == 1:
            result[originals[0]] = base
            continue
        for original in originals:
            prefix = base[:130].rstrip(" ._") or fallback
            result[original] = "{}__{}".format(prefix, _short_hash(original))
    return result


def default_output_directory(bag_path: Path) -> Path:
    """Return the deterministic output directory beside a bag file or directory."""

    bag_name = bag_path.name if bag_path.is_dir() else bag_path.stem
    return bag_path.parent / "{}_compute_times".format(bag_name)
