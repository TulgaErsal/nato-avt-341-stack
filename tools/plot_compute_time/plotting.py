"""Matplotlib rendering for hierarchical compute-time samples."""

from pathlib import Path
from typing import Any, List, Mapping, Sequence

import matplotlib

# This utility only writes files and must also work on machines without a display server.
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (backend must be selected first)

from .models import (
    ComputeSample,
    direct_children,
    parse_section_path,
    unique_safe_names,
)


def _plot_series(
    axes: Any,
    section_id: str,
    samples: Sequence[ComputeSample],
    color: Any,
    linewidth: float,
    band_alpha: float,
) -> None:
    ordered = sorted(samples, key=lambda sample: sample.relative_time_seconds)
    times = [sample.relative_time_seconds for sample in ordered]
    means = [sample.mean_milliseconds for sample in ordered]
    lower = [
        max(0.0, sample.mean_milliseconds - sample.std_milliseconds)
        for sample in ordered
    ]
    upper = [
        sample.mean_milliseconds + sample.std_milliseconds for sample in ordered
    ]
    synthesized = any(sample.synthesized for sample in ordered)
    label = "{} (mean +/- 1 std{})".format(
        section_id, ", synthesized" if synthesized else ""
    )
    axes.plot(
        times,
        means,
        color=color,
        linewidth=linewidth,
        marker=".",
        markersize=3,
        label=label,
    )
    axes.fill_between(times, lower, upper, color=color, alpha=band_alpha, linewidth=0)


def create_section_figure(
    vehicle_id: str,
    tag: str,
    section_id: str,
    section_samples: Mapping[str, Sequence[ComputeSample]],
) -> Any:
    """Create one section figure, overlaying only its direct children."""

    if section_id not in section_samples:
        raise ValueError("Unknown section_id: {}".format(section_id))

    children = direct_children(section_id, section_samples.keys())
    figure, axes = plt.subplots(figsize=(12, 6))
    _plot_series(
        axes,
        section_id,
        section_samples[section_id],
        color="black" if children else "tab:blue",
        linewidth=2.2,
        band_alpha=0.18,
    )

    color_map = plt.get_cmap("tab10")
    for index, child_id in enumerate(children):
        _plot_series(
            axes,
            child_id,
            section_samples[child_id],
            color=color_map(index % 10),
            linewidth=1.4,
            band_alpha=0.10,
        )

    tag_label = tag if tag else "<untagged>"
    axes.set_title("Vehicle: {} | Tag: {}\nSection: {}".format(vehicle_id, tag_label, section_id))
    axes.set_xlabel("Bag time since first selected compute-time message (s)")
    axes.set_ylabel("Compute time (ms)")
    axes.grid(True, alpha=0.3)
    axes.legend(loc="best")
    figure.tight_layout()
    return figure


def render_plots(
    grouped: Mapping[str, Mapping[str, Mapping[str, Sequence[ComputeSample]]]],
    output_root: Path,
) -> List[Path]:
    """Render all observed sections into the vehicle/tag/flat-section layout."""

    vehicle_names = unique_safe_names(grouped.keys(), "vehicle")
    saved_paths: List[Path] = []
    output_root.mkdir(parents=True, exist_ok=True)

    for vehicle_id in sorted(grouped):
        tags = grouped[vehicle_id]
        tag_names = unique_safe_names(tags.keys(), "untagged")
        vehicle_directory = output_root / vehicle_names[vehicle_id]

        for tag in sorted(tags):
            sections = tags[tag]
            section_names = unique_safe_names(sections.keys(), "section")
            tag_directory = vehicle_directory / tag_names[tag]
            tag_directory.mkdir(parents=True, exist_ok=True)

            for section_id in sorted(sections, key=parse_section_path):
                figure = create_section_figure(vehicle_id, tag, section_id, sections)
                output_path = tag_directory / "{}.png".format(section_names[section_id])
                try:
                    figure.savefig(output_path, dpi=150)
                finally:
                    plt.close(figure)
                saved_paths.append(output_path)

    return saved_paths

