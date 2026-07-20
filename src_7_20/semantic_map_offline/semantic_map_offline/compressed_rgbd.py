"""Decode ROS compressed RGB and 16-bit compressedDepth payloads."""

from __future__ import annotations

import cv2
import numpy as np


PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def decode_color(data: bytes) -> np.ndarray:
    """Decode a JPEG/PNG CompressedImage payload as BGR8."""
    encoded = np.frombuffer(bytes(data), dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("OpenCV could not decode compressed color image")
    return image


def decode_16uc1_depth(data: bytes, image_format: str) -> np.ndarray:
    """Extract and decode the PNG stored after a compressedDepth header."""
    if "16UC1" not in image_format:
        raise ValueError(
            f"Only 16UC1 compressedDepth is supported, got {image_format!r}"
        )
    payload = bytes(data)
    png_offset = payload.find(PNG_SIGNATURE)
    if png_offset < 0:
        raise ValueError("compressedDepth payload has no PNG signature")
    encoded = np.frombuffer(payload[png_offset:], dtype=np.uint8)
    depth = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise ValueError("OpenCV could not decode compressed depth image")
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError(
            f"Expected a uint16 depth image, got shape={depth.shape}, dtype={depth.dtype}"
        )
    return depth


def standard_optical_quaternion() -> tuple[float, float, float, float]:
    """Return lidar/body XYZ-forward-left-up to optical right-down-forward."""
    return -0.5, 0.5, -0.5, 0.5
