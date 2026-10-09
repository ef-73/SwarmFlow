"""Command line entry point."""

from __future__ import annotations

import argparse
import sys

from .generate import LayoutError, generate_all, load_layout


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Generate all artefacts of a SwarmFlow layout.")
    ap.add_argument("layout", help="layouts/<name>/layout.yaml")
    ap.add_argument("--out", required=True, help="output directory")
    args = ap.parse_args(argv)
    try:
        layout = load_layout(args.layout)
        generate_all(layout, args.out)
    except (LayoutError, OSError, ValueError) as e:
        print(f"layoutgen: error: {e}", file=sys.stderr)
        return 1
    return 0
