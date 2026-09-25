#!/usr/bin/env python3

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nanolite_run_view import parse_latest_snapshots, reconstruct_odometry, render_run


def packed_empty_map(size: int) -> str:
    return bytes((size * size + 3) // 4).hex()


class NanoLiteRunViewTests(unittest.TestCase):
    def setUp(self) -> None:
        cells = packed_empty_map(4)
        self.log = "\n".join(
            [
                f"I nanolite: NANOMAP v=1 seq=7 anchor=1.00,2.00 robot=2,2 "
                f"size=4 res_cm=50 rays=42 cells={cells}",
                "I nanolite: NANOGRAPH_BEGIN v=1 seq=7 poses=3 odom=2 loops=1 "
                "live=1.500,2.250,0.10000",
                "I nanolite: NANONODE v=1 seq=7 i=0 x=1.000 y=2.000 "
                "yaw=0.00000 t_us=100 flags=0x01",
                "I nanolite: NANONODE v=1 seq=7 i=1 x=1.300 y=2.000 "
                "yaw=0.00000 t_us=200 flags=0x03",
                "I nanolite: NANONODE v=1 seq=7 i=2 x=1.500 y=2.200 "
                "yaw=0.10000 t_us=300 flags=0x05",
                "I nanolite: NANOEDGE v=1 seq=7 kind=odom i=0 from=0 to=1 "
                "dx=0.300 dy=0.000 dyaw=0.00000",
                "I nanolite: NANOEDGE v=1 seq=7 kind=odom i=1 from=1 to=2 "
                "dx=0.250 dy=0.200 dyaw=0.10000",
                "I nanolite: NANOEDGE v=1 seq=7 kind=loop i=0 from=2 to=0 "
                "dx=-0.450 dy=-0.200 dyaw=-0.10000",
                "I nanolite: NANOGRAPH_END v=1 seq=7",
            ]
        )

    def test_parse_complete_snapshot(self) -> None:
        map_snapshot, graph = parse_latest_snapshots(self.log)
        self.assertEqual(map_snapshot.sequence, 7)
        self.assertEqual(map_snapshot.rays, 42)
        self.assertEqual(len(graph.nodes), 3)
        self.assertEqual(len(graph.odometry), 2)
        self.assertEqual(len(graph.loops), 1)
        self.assertAlmostEqual(graph.live_x_m, 1.5)

    def test_reconstructs_raw_odometry_from_anchor(self) -> None:
        _, graph = parse_latest_snapshots(self.log)
        raw = reconstruct_odometry(graph)
        self.assertAlmostEqual(raw[1][0], 1.3)
        self.assertAlmostEqual(raw[1][1], 2.0)
        self.assertAlmostEqual(raw[2][0], 1.55)
        self.assertAlmostEqual(raw[2][1], 2.2)

    def test_incomplete_graph_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "no complete NANOGRAPH"):
            parse_latest_snapshots(self.log.replace("NANOGRAPH_END", "BROKEN_END"))

    def test_png_render(self) -> None:
        map_snapshot, graph = parse_latest_snapshots(self.log)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run.png"
            render_run(map_snapshot, graph, output, dpi=80)
            self.assertGreater(output.stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
