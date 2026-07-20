#!/usr/bin/env python3
"""Track and fuse MobileSAM-segmented Replica objects."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_map_offline.tracking_pipeline import build_common_parser, run_tracking


def main() -> None:
    parser = build_common_parser()
    parser.add_argument("--sam-checkpoint", type=Path, required=True)
    parser.add_argument("--sam-source", type=Path, default=Path(__file__).resolve().parents[1] / "MobileSAM")
    parser.add_argument("--sam-device", default="")
    parser.add_argument("--mask-erode-px", type=int, default=2)
    args = parser.parse_args()
    run_tracking(args, {
        "checkpoint": args.sam_checkpoint,
        "source": args.sam_source,
        "device": args.sam_device,
        "mask_erode_px": args.mask_erode_px,
    })


if __name__ == "__main__":
    main()
