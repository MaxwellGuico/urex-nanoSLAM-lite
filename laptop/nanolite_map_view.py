#!/usr/bin/env python3
"""Render the latest packed NANOMAP record from an ESP-IDF monitor log."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


MAP_RE = re.compile(
    r"NANOMAP v=(?P<version>\d+) seq=(?P<sequence>\d+) "
    r"anchor=(?P<anchor_x>-?[\d.]+),(?P<anchor_y>-?[\d.]+) "
    r"robot=(?P<robot_x>-?\d+),(?P<robot_y>-?\d+) "
    r"size=(?P<size>\d+) res_cm=(?P<resolution>\d+) "
    r"rays=(?P<rays>\d+) cells=(?P<cells>[0-9a-fA-F]+)"
)

CELL_CHARS = "?.+#"


def decode_cells(packed_hex: str, size: int) -> list[int]:
    packed = bytes.fromhex(packed_hex)
    expected = (size * size + 3) // 4
    if len(packed) != expected:
        raise ValueError(f"expected {expected} packed bytes, received {len(packed)}")
    return [(packed[index >> 2] >> ((index & 3) * 2)) & 3
            for index in range(size * size)]


def render(match: re.Match[str]) -> str:
    fields = match.groupdict()
    size = int(fields["size"])
    cells = decode_cells(fields["cells"], size)
    robot_x = int(fields["robot_x"])
    robot_y = int(fields["robot_y"])
    anchor_x = size // 2
    anchor_y = size // 2

    known = [(index % size, index // size)
             for index, value in enumerate(cells) if value != 0]
    known.extend([(anchor_x, anchor_y)])
    if 0 <= robot_x < size and 0 <= robot_y < size:
        known.append((robot_x, robot_y))
    min_x = max(0, min(x for x, _ in known) - 1)
    max_x = min(size - 1, max(x for x, _ in known) + 1)
    min_y = max(0, min(y for _, y in known) - 1)
    max_y = min(size - 1, max(y for _, y in known) + 1)

    lines = [
        f"NanoSLAM-Lite map seq={fields['sequence']} rays={fields['rays']} "
        f"anchor=({fields['anchor_x']},{fields['anchor_y']}) "
        f"resolution={fields['resolution']}cm",
        "legend: ? unknown  . free  + tentative hit  # occupied  A anchor  R robot",
    ]
    for y in range(max_y, min_y - 1, -1):
        row = []
        for x in range(min_x, max_x + 1):
            if x == robot_x and y == robot_y:
                row.append("R")
            elif x == anchor_x and y == anchor_y:
                row.append("A")
            else:
                row.append(CELL_CHARS[cells[y * size + x]])
        lines.append("".join(row))
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", help="monitor log; omit to read stdin")
    args = parser.parse_args()
    text = Path(args.log).read_text(errors="replace") if args.log else sys.stdin.read()
    matches = list(MAP_RE.finditer(text))
    if not matches:
        print("No complete NANOMAP record found.", file=sys.stderr)
        return 1
    print(render(matches[-1]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
