"""Core utilities for building a 2D semantic map on top of a ROS occupancy map.

The package intentionally keeps projection and fusion logic independent from ROS so
it can be unit-tested on development machines and reused by ROS 2 nodes.
"""

from semantic_map_pkg.config import DEFAULT_SEMANTIC_CLASSES, SemanticClass, resolve_semantic_class
from semantic_map_pkg.fusion import SemanticObject, SemanticObjectTracker
from semantic_map_pkg.projection import Detection2D, ProjectedObject, SemanticProjector
from semantic_map_pkg.visualization import Rectangle2D, object_to_rectangle, rectangle_corners

__all__ = [
    "DEFAULT_SEMANTIC_CLASSES",
    "Detection2D",
    "ProjectedObject",
    "SemanticClass",
    "SemanticObject",
    "SemanticObjectTracker",
    "SemanticProjector",
    "Rectangle2D",
    "object_to_rectangle",
    "rectangle_corners",
    "resolve_semantic_class",
]
