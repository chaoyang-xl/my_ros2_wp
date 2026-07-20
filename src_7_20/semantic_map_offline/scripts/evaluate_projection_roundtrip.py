#!/usr/bin/env python3
"""Track and fuse bbox-projected Replica objects without segmentation."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_map_offline.tracking_pipeline import build_common_parser, run_tracking


def main() -> None:
    parser = build_common_parser()
    args = parser.parse_args()
    run_tracking(args)


if __name__ == "__main__":
    main()
