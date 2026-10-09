#!/usr/bin/env python3
"""CLI: generate.py <layout.yaml> --out <dir>  (design §6.1)."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from layoutgen.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
