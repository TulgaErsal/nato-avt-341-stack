"""Matplotlib rendering for hierarchical compute-time samples."""

import re
from pathlib import Path
from typing import Any, List, Mapping, Sequence, Tuple

import numpy as np

import matplotlib

# This utility only writes files and must also work on machines without a display server.
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (backend must be selected first)

from .models import (  # noqa: E402
    ComputeSample,
    direct_children,
    parse_section_path,
    unique_safe_names,
)

matplotlib.rcParams.update(
    {
        "font.family": "serif",
        "font.size": 13.0,
        "axes.titlesize": 15.0,
        "axes.labelsize": 14.0,
        "xtick.labelsize": 12.0,
        "ytick.labelsize": 12.0,
        "legend.fontsize": 12.0,
        "figure.titlesize": 17.0,
        "mathtext.fontset": "dejavuserif",
        # No horizontal padding: data spans the full x-axis width.
        "axes.xmargin": 0.0,
    }
)

_OVERLAY_SUBTITLE = "Compute Times ($\\mu \\pm \\sigma$)"
_STACKED_SUBTITLE = "Stacked Compute Times ($\\mu \\pm \\sigma$)"

_MATHTEXT_SAFE = re.compile(r"[A-Za-z0-9_/.\- ]+\Z")


def _bold_title_text(text: str) -> str:
    """Return text bolded via mathtext when its characters allow, else unchanged."""

    if not _MATHTEXT_SAFE.match(text):
        return text
    escaped = text.replace("_", r"\_").replace(" ", r"\ ")
    return r"$\mathbf{" + escaped + r"}$"


def _sorted_arrays(samples: Sequence[ComputeSample]) -> Tuple[Any, Any, Any, bool]:
    """Return time-sorted numpy arrays (times, means, stds) and a synthesized flag."""

    ordered = sorted(samples, key=lambda sample: sample.relative_time_seconds)
    times = np.array([sample.relative_time_seconds for sample in ordered], dtype=float)
    means = np.array([sample.mean_milliseconds for sample in ordered], dtype=float)
    stds = np.array([sample.std_milliseconds for sample in ordered], dtype=float)
    synthesized = any(sample.synthesized for sample in ordered)
    return times, means, stds, synthesized


def _plot_series(
    axes: Any,
    legend_label: str,
    samples: Sequence[ComputeSample],
    color: Any,
    linewidth: float,
    band_alpha: float,
) -> None:
    times, means, stds, synthesized = _sorted_arrays(samples)
    label = "{}{}".format(legend_label, " (synthesized)" if synthesized else "")
    axes.plot(
        times,
        means,
        color=color,
        linewidth=linewidth,
        marker=".",
        markersize=3,
        label=label,
    )
    axes.fill_between(
        times,
        np.maximum(0.0, means - stds),
        means + stds,
        color=color,
        alpha=band_alpha,
        linewidth=0,
    )


def _draw_stacked(
    axes: Any,
    section_id: str,
    section_samples: Mapping[str, Sequence[ComputeSample]],
    children: Sequence[str],
) -> None:
    """Draw the cumulative child stack with the parent's measured total on top.

    Children stack in descending-mean order (largest at the bottom) while each
    child keeps the color of its position in the given children sequence.
    """

    parent_times, parent_means, parent_stds, _ = _sorted_arrays(
        section_samples[section_id]
    )
    color_map = plt.get_cmap("tab10")
    child_colors = {
        child_id: color_map(index % 10) for index, child_id in enumerate(children)
    }
    child_series = {}
    for child_id in children:
        times, means, stds, _ = _sorted_arrays(section_samples[child_id])
        if len(times) != len(parent_times) or not np.allclose(times, parent_times):
            means = np.interp(parent_times, times, means)
            stds = np.interp(parent_times, times, stds)
        child_series[child_id] = (means, stds)
    stack_order = sorted(
        children,
        key=lambda child_id: float(np.mean(child_series[child_id][0])),
        reverse=True,
    )

    cumulative = np.zeros_like(parent_means)
    for child_id in stack_order:
        means, stds = child_series[child_id]
        color = child_colors[child_id]
        stacked = cumulative + means
        axes.fill_between(
            parent_times, cumulative, stacked, color=color, alpha=0.45, linewidth=0
        )
        axes.plot(
            parent_times, stacked, color=color, linewidth=1.4, marker=".", markersize=3
        )
        # The band around each stack boundary is that child's own +/- 1 std.
        axes.fill_between(
            parent_times,
            np.maximum(0.0, stacked - stds),
            stacked + stds,
            color=color,
            alpha=0.30,
            linewidth=0,
        )
        cumulative = stacked

    axes.plot(
        parent_times, parent_means, color="black", linewidth=2.2, marker=".", markersize=3
    )
    axes.fill_between(
        parent_times,
        np.maximum(0.0, parent_means - parent_stds),
        parent_means + parent_stds,
        color="black",
        alpha=0.18,
        linewidth=0,
    )


def _style_axes(axes: Any, subtitle: str) -> None:
    axes.set_title(subtitle)
    axes.set_ylabel("Compute time (ms)")
    axes.minorticks_on()
    axes.grid(True, which="major", alpha=0.35)
    axes.grid(True, which="minor", alpha=0.15)
    # No lower margin: the y axis starts at the lowest plotted value, while the
    # upper limit keeps the default autoscale margin.
    axes.set_ylim(bottom=axes.dataLim.y0)
    # Anchor the x axis at bag start; xmargin=0 already ends it at the last sample.
    axes.set_xlim(left=0.0)


def _save_figure(figure: Any, output_path: Path) -> Path:
    try:
        figure.savefig(output_path, dpi=150, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output_path


def create_section_figure(
    tag: str,
    section_id: str,
    section_samples: Mapping[str, Sequence[ComputeSample]],
) -> Any:
    """Create one section figure: an overlay subplot plus a stacked subplot when
    the section has direct children; leaf sections get the overlay alone."""

    if section_id not in section_samples:
        raise ValueError("Unknown section_id: {}".format(section_id))

    children = direct_children(section_id, section_samples.keys())
    # Legend labels are relative to the plotted section's level in the hierarchy.
    label_start = len(parse_section_path(section_id)) - 1

    def relative_label(full_section_id: str) -> str:
        return "/".join(parse_section_path(full_section_id)[label_start:])

    rows = 2 if children else 1
    figure, axes_grid = plt.subplots(
        rows, 1, figsize=(12, 5.5 * rows), sharex=rows > 1, squeeze=False
    )
    overlay_axes = axes_grid[0][0]

    _plot_series(
        overlay_axes,
        relative_label(section_id),
        section_samples[section_id],
        color="black" if children else "tab:blue",
        linewidth=2.2,
        band_alpha=0.18,
    )
    color_map = plt.get_cmap("tab10")
    for index, child_id in enumerate(children):
        _plot_series(
            overlay_axes,
            relative_label(child_id),
            section_samples[child_id],
            color=color_map(index % 10),
            linewidth=1.4,
            band_alpha=0.10,
        )
    _style_axes(overlay_axes, _OVERLAY_SUBTITLE)

    if children:
        stacked_axes = axes_grid[1][0]
        _draw_stacked(stacked_axes, section_id, section_samples, children)
        _style_axes(stacked_axes, _STACKED_SUBTITLE)

    bottom_axes = axes_grid[-1][0]
    bottom_axes.set_xlabel("Bag time (s)")
    handles, labels = overlay_axes.get_legend_handles_labels()
    bottom_axes.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.15),
        ncol=max(1, min(len(labels), 5)),
        frameon=False,
    )

    tag_label = tag if tag else "<untagged>"
    # Fixed-inch spacing: the suptitle sits 0.05in below the top edge and the
    # subplot area starts 0.68in down, leaving a small constant gap between the
    # suptitle and the first subplot title at any figure height.
    figure_height = 5.5 * rows
    figure.tight_layout()
    figure.subplots_adjust(top=1.0 - 0.68 / figure_height)
    figure.suptitle(
        "{}: {}".format(tag_label, _bold_title_text(section_id)),
        y=1.0 - 0.05 / figure_height,
    )
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
                figure = create_section_figure(tag, section_id, sections)
                saved_paths.append(
                    _save_figure(
                        figure,
                        tag_directory / "{}.png".format(section_names[section_id]),
                    )
                )

    return saved_paths
