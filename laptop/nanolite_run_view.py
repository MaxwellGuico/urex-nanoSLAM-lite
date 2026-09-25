#!/usr/bin/env python3
"""Render a NanoSLAM-Lite occupancy map and pose graph from a monitor log."""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path


NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"

MAP_RE = re.compile(
    rf"NANOMAP v=(?P<version>\d+) seq=(?P<sequence>\d+) "
    rf"anchor=(?P<anchor_x>{NUMBER}),(?P<anchor_y>{NUMBER}) "
    r"robot=(?P<robot_x>-?\d+),(?P<robot_y>-?\d+) "
    r"size=(?P<size>\d+) res_cm=(?P<resolution>\d+) "
    r"rays=(?P<rays>\d+) cells=(?P<cells>[0-9a-fA-F]+)"
)
GRAPH_BEGIN_RE = re.compile(
    rf"NANOGRAPH_BEGIN v=(?P<version>\d+) seq=(?P<sequence>\d+) "
    r"poses=(?P<poses>\d+) odom=(?P<odom>\d+) loops=(?P<loops>\d+) "
    rf"live=(?P<live_x>{NUMBER}),(?P<live_y>{NUMBER}),"
    rf"(?P<live_yaw>{NUMBER})"
)
NODE_RE = re.compile(
    rf"NANONODE v=(?P<version>\d+) seq=(?P<sequence>\d+) "
    rf"i=(?P<index>\d+) x=(?P<x>{NUMBER}) y=(?P<y>{NUMBER}) "
    rf"yaw=(?P<yaw>{NUMBER}) t_us=(?P<timestamp>\d+) "
    r"flags=0x(?P<flags>[0-9a-fA-F]+)"
)
EDGE_RE = re.compile(
    rf"NANOEDGE v=(?P<version>\d+) seq=(?P<sequence>\d+) "
    r"kind=(?P<kind>odom|loop) i=(?P<index>\d+) "
    r"from=(?P<from_index>\d+) to=(?P<to_index>\d+) "
    rf"dx=(?P<dx>{NUMBER}) dy=(?P<dy>{NUMBER}) "
    rf"dyaw=(?P<dyaw>{NUMBER})"
)
GRAPH_END_RE = re.compile(
    r"NANOGRAPH_END v=(?P<version>\d+) seq=(?P<sequence>\d+)"
)

POSE_SCAN = 1 << 1
POSE_CORRECTED = 1 << 2


@dataclass(frozen=True)
class MapSnapshot:
    sequence: int
    anchor_x_m: float
    anchor_y_m: float
    robot_x: int
    robot_y: int
    size: int
    resolution_m: float
    rays: int
    cells: tuple[int, ...]


@dataclass(frozen=True)
class Node:
    index: int
    x_m: float
    y_m: float
    yaw_rad: float
    timestamp_us: int
    flags: int


@dataclass(frozen=True)
class Edge:
    index: int
    kind: str
    from_index: int
    to_index: int
    dx_m: float
    dy_m: float
    dyaw_rad: float


@dataclass
class GraphSnapshot:
    sequence: int
    expected_poses: int
    expected_odometry: int
    expected_loops: int
    live_x_m: float
    live_y_m: float
    live_yaw_rad: float
    nodes: dict[int, Node] = field(default_factory=dict)
    odometry: list[Edge] = field(default_factory=list)
    loops: list[Edge] = field(default_factory=list)


def decode_cells(packed_hex: str, size: int) -> tuple[int, ...]:
    packed = bytes.fromhex(packed_hex)
    expected = (size * size + 3) // 4
    if len(packed) != expected:
        raise ValueError(
            f"map requires {expected} packed bytes, received {len(packed)}"
        )
    return tuple(
        (packed[index >> 2] >> ((index & 3) * 2)) & 3
        for index in range(size * size)
    )


def _parse_map(match: re.Match[str]) -> MapSnapshot:
    fields = match.groupdict()
    if int(fields["version"]) != 1:
        raise ValueError(f"unsupported NANOMAP version {fields['version']}")
    size = int(fields["size"])
    return MapSnapshot(
        sequence=int(fields["sequence"]),
        anchor_x_m=float(fields["anchor_x"]),
        anchor_y_m=float(fields["anchor_y"]),
        robot_x=int(fields["robot_x"]),
        robot_y=int(fields["robot_y"]),
        size=size,
        resolution_m=int(fields["resolution"]) / 100.0,
        rays=int(fields["rays"]),
        cells=decode_cells(fields["cells"], size),
    )


def _parse_graph_begin(match: re.Match[str]) -> GraphSnapshot:
    fields = match.groupdict()
    if int(fields["version"]) != 1:
        raise ValueError(f"unsupported NANOGRAPH version {fields['version']}")
    return GraphSnapshot(
        sequence=int(fields["sequence"]),
        expected_poses=int(fields["poses"]),
        expected_odometry=int(fields["odom"]),
        expected_loops=int(fields["loops"]),
        live_x_m=float(fields["live_x"]),
        live_y_m=float(fields["live_y"]),
        live_yaw_rad=float(fields["live_yaw"]),
    )


def _graph_is_complete(graph: GraphSnapshot) -> bool:
    return (
        len(graph.nodes) == graph.expected_poses
        and set(graph.nodes) == set(range(graph.expected_poses))
        and len(graph.odometry) == graph.expected_odometry
        and len(graph.loops) == graph.expected_loops
    )


def parse_latest_snapshots(text: str) -> tuple[MapSnapshot, GraphSnapshot]:
    """Return the latest complete map and matching complete graph snapshot."""
    maps: list[tuple[int, MapSnapshot]] = []
    graphs: list[tuple[int, GraphSnapshot]] = []
    current: GraphSnapshot | None = None

    for line_number, line in enumerate(text.splitlines()):
        map_match = MAP_RE.search(line)
        if map_match:
            maps.append((line_number, _parse_map(map_match)))
            continue

        begin_match = GRAPH_BEGIN_RE.search(line)
        if begin_match:
            current = _parse_graph_begin(begin_match)
            continue

        node_match = NODE_RE.search(line)
        if node_match and current is not None:
            fields = node_match.groupdict()
            if int(fields["version"]) == 1 and int(fields["sequence"]) == current.sequence:
                index = int(fields["index"])
                current.nodes[index] = Node(
                    index=index,
                    x_m=float(fields["x"]),
                    y_m=float(fields["y"]),
                    yaw_rad=float(fields["yaw"]),
                    timestamp_us=int(fields["timestamp"]),
                    flags=int(fields["flags"], 16),
                )
            continue

        edge_match = EDGE_RE.search(line)
        if edge_match and current is not None:
            fields = edge_match.groupdict()
            if int(fields["version"]) == 1 and int(fields["sequence"]) == current.sequence:
                edge = Edge(
                    index=int(fields["index"]),
                    kind=fields["kind"],
                    from_index=int(fields["from_index"]),
                    to_index=int(fields["to_index"]),
                    dx_m=float(fields["dx"]),
                    dy_m=float(fields["dy"]),
                    dyaw_rad=float(fields["dyaw"]),
                )
                (current.odometry if edge.kind == "odom" else current.loops).append(edge)
            continue

        end_match = GRAPH_END_RE.search(line)
        if end_match and current is not None:
            fields = end_match.groupdict()
            if (
                int(fields["version"]) == 1
                and int(fields["sequence"]) == current.sequence
                and _graph_is_complete(current)
            ):
                current.odometry.sort(key=lambda edge: edge.index)
                current.loops.sort(key=lambda edge: edge.index)
                graphs.append((line_number, current))
            current = None

    if not maps:
        raise ValueError("no complete NANOMAP record found")
    if not graphs:
        raise ValueError(
            "no complete NANOGRAPH snapshot found; flash firmware with "
            "CONFIG_NANOLITE_GRAPH_SERIAL_DUMP enabled"
        )

    map_line, map_snapshot = maps[-1]
    matching = [
        (line_number, graph)
        for line_number, graph in graphs
        if graph.sequence == map_snapshot.sequence
    ]
    if matching:
        # A graph is emitted immediately after its map. Selecting the nearest
        # record also handles logs containing more than one device reboot.
        _, graph_snapshot = min(
            matching, key=lambda item: abs(item[0] - map_line)
        )
    else:
        _, graph_snapshot = graphs[-1]
    return map_snapshot, graph_snapshot


def reconstruct_odometry(graph: GraphSnapshot) -> dict[int, tuple[float, float, float]]:
    """Integrate immutable odometry constraints from the corrected anchor."""
    if 0 not in graph.nodes:
        return {}
    anchor = graph.nodes[0]
    raw: dict[int, tuple[float, float, float]] = {
        0: (anchor.x_m, anchor.y_m, anchor.yaw_rad)
    }
    for edge in graph.odometry:
        if edge.from_index not in raw:
            continue
        x_m, y_m, yaw_rad = raw[edge.from_index]
        cosine = math.cos(yaw_rad)
        sine = math.sin(yaw_rad)
        raw[edge.to_index] = (
            x_m + cosine * edge.dx_m - sine * edge.dy_m,
            y_m + sine * edge.dx_m + cosine * edge.dy_m,
            math.atan2(
                math.sin(yaw_rad + edge.dyaw_rad),
                math.cos(yaw_rad + edge.dyaw_rad),
            ),
        )
    return raw


def render_run(
    map_snapshot: MapSnapshot,
    graph: GraphSnapshot,
    output: Path,
    dpi: int = 160,
) -> None:
    # Import lazily so parsing and tests do not require a plotting backend.
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    size = map_snapshot.size
    grid = np.asarray(map_snapshot.cells, dtype=np.uint8).reshape(size, size)
    half_span = size * map_snapshot.resolution_m * 0.5
    east_min = map_snapshot.anchor_y_m - half_span
    east_max = map_snapshot.anchor_y_m + half_span
    north_min = map_snapshot.anchor_x_m - half_span
    north_max = map_snapshot.anchor_x_m + half_span

    fig, (map_axis, graph_axis) = plt.subplots(1, 2, figsize=(15, 7.5))
    colors = ListedColormap(["#d9dde3", "#ffffff", "#f3b562", "#1f2933"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], colors.N)
    map_axis.imshow(
        grid.T,
        origin="lower",
        interpolation="nearest",
        extent=[east_min, east_max, north_min, north_max],
        cmap=colors,
        norm=norm,
    )

    ordered_nodes = [graph.nodes[index] for index in sorted(graph.nodes)]
    corrected_east = [node.y_m for node in ordered_nodes]
    corrected_north = [node.x_m for node in ordered_nodes]
    map_axis.plot(
        corrected_east,
        corrected_north,
        color="#1976d2",
        linewidth=1.8,
        marker="o",
        markersize=3,
        label="corrected key poses",
    )
    map_axis.scatter(
        [map_snapshot.anchor_y_m],
        [map_snapshot.anchor_x_m],
        marker="X",
        s=80,
        color="#2e7d32",
        edgecolor="white",
        linewidth=0.8,
        zorder=6,
        label="anchor",
    )
    map_axis.scatter(
        [graph.live_y_m],
        [graph.live_x_m],
        marker="*",
        s=130,
        color="#d32f2f",
        edgecolor="white",
        linewidth=0.8,
        zorder=7,
        label="live pose",
    )
    map_axis.set_title(
        f"Occupancy map - snapshot {map_snapshot.sequence}\n"
        f"{map_snapshot.rays} integrated rays"
    )
    map_axis.set_xlabel("East (m)")
    map_axis.set_ylabel("North (m)")
    map_axis.grid(color="#607d8b", alpha=0.18, linewidth=0.5)
    map_axis.set_aspect("equal", adjustable="box")
    map_legend = [
        Patch(facecolor="#d9dde3", label="unknown"),
        Patch(facecolor="#ffffff", edgecolor="#9aa4af", label="free"),
        Patch(facecolor="#f3b562", label="tentative hit"),
        Patch(facecolor="#1f2933", label="occupied"),
        Line2D([0], [0], color="#1976d2", marker="o",
               label="corrected key poses"),
        Line2D([0], [0], marker="X", linestyle="none", color="#2e7d32",
               label="anchor"),
        Line2D([0], [0], marker="*", linestyle="none", color="#d32f2f",
               markersize=10, label="live pose"),
    ]
    map_axis.legend(handles=map_legend, loc="upper right", fontsize=8, ncol=2)

    raw = reconstruct_odometry(graph)
    raw_indices = sorted(raw)
    if raw_indices:
        graph_axis.plot(
            [raw[index][1] for index in raw_indices],
            [raw[index][0] for index in raw_indices],
            linestyle="--",
            linewidth=1.5,
            color="#8b95a1",
            label="raw odometry integration",
        )

    graph_axis.plot(
        corrected_east,
        corrected_north,
        color="#1976d2",
        linewidth=2.0,
        marker="o",
        markersize=4,
        label="optimized graph",
    )
    for edge in graph.loops:
        if edge.from_index not in graph.nodes or edge.to_index not in graph.nodes:
            continue
        start = graph.nodes[edge.from_index]
        end = graph.nodes[edge.to_index]
        graph_axis.plot(
            [start.y_m, end.y_m],
            [start.x_m, end.x_m],
            color="#c2185b",
            linewidth=2.2,
            linestyle=":",
            zorder=4,
        )

    scan_nodes = [node for node in ordered_nodes if node.flags & POSE_SCAN]
    if scan_nodes:
        graph_axis.scatter(
            [node.y_m for node in scan_nodes],
            [node.x_m for node in scan_nodes],
            marker="s",
            s=38,
            color="#43a047",
            edgecolor="white",
            linewidth=0.6,
            zorder=5,
        )
    corrected_nodes = [node for node in ordered_nodes if node.flags & POSE_CORRECTED]
    if corrected_nodes:
        graph_axis.scatter(
            [node.y_m for node in corrected_nodes],
            [node.x_m for node in corrected_nodes],
            marker="o",
            s=65,
            facecolor="none",
            edgecolor="#ef6c00",
            linewidth=1.2,
            zorder=5,
        )

    arrow_stride = max(1, len(ordered_nodes) // 16)
    arrow_nodes = ordered_nodes[::arrow_stride]
    if arrow_nodes:
        graph_axis.quiver(
            [node.y_m for node in arrow_nodes],
            [node.x_m for node in arrow_nodes],
            [math.sin(node.yaw_rad) for node in arrow_nodes],
            [math.cos(node.yaw_rad) for node in arrow_nodes],
            angles="xy",
            scale_units="xy",
            scale=4.0,
            width=0.004,
            color="#0d47a1",
            alpha=0.8,
        )
    for node in ordered_nodes:
        graph_axis.annotate(
            str(node.index),
            (node.y_m, node.x_m),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=7,
            color="#263238",
        )
    graph_axis.scatter(
        [graph.live_y_m],
        [graph.live_x_m],
        marker="*",
        s=130,
        color="#d32f2f",
        edgecolor="white",
        linewidth=0.8,
        zorder=7,
    )

    legend_items = [
        Line2D([0], [0], color="#1976d2", marker="o", label="optimized graph"),
        Line2D([0], [0], color="#8b95a1", linestyle="--", label="raw odometry"),
        Line2D([0], [0], color="#c2185b", linestyle=":", linewidth=2.2,
               label="loop closure"),
        Line2D([0], [0], marker="s", linestyle="none", color="#43a047",
               label="scan pose"),
        Line2D([0], [0], marker="o", linestyle="none", markerfacecolor="none",
               markeredgecolor="#ef6c00", label="optimizer changed"),
        Line2D([0], [0], marker="*", linestyle="none", color="#d32f2f",
               markersize=11, label="live pose"),
    ]
    graph_axis.legend(handles=legend_items, loc="best", fontsize=8)
    graph_axis.set_title(
        "Pose graph\n"
        f"{len(graph.nodes)} nodes, {len(graph.odometry)} odometry edges, "
        f"{len(graph.loops)} loop edges"
    )
    graph_axis.set_xlabel("East (m)")
    graph_axis.set_ylabel("North (m)")
    graph_axis.grid(alpha=0.25)
    graph_axis.set_aspect("equal", adjustable="datalim")

    counts = [map_snapshot.cells.count(value) for value in range(4)]
    fig.suptitle(
        "NanoSLAM-Lite run snapshot\n"
        f"map cells: unknown={counts[0]}, free={counts[1]}, "
        f"candidate={counts[2]}, occupied={counts[3]}",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="ESP-IDF monitor log containing snapshots")
    parser.add_argument(
        "-o",
        "--output",
        help="output PNG (default: LOG-nanolite.png)",
    )
    parser.add_argument("--dpi", type=int, default=160, help="PNG resolution")
    args = parser.parse_args()

    log_path = Path(args.log)
    output_path = (
        Path(args.output)
        if args.output
        else log_path.with_name(f"{log_path.stem}-nanolite.png")
    )
    try:
        map_snapshot, graph_snapshot = parse_latest_snapshots(
            log_path.read_text(errors="replace")
        )
        render_run(map_snapshot, graph_snapshot, output_path, args.dpi)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    print(
        f"rendered snapshot {map_snapshot.sequence}: "
        f"{len(graph_snapshot.nodes)} poses, "
        f"{len(graph_snapshot.loops)} loop edges -> {output_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
