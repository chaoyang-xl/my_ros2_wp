"""Semantic class configuration used by projection and visualization code."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class SemanticClass:
    """Display and geometry defaults for a semantic object category.

    Attributes:
        canonical_label: Stable internal label used by APIs and storage.
        display_label: Human-readable label for RViz/debug overlays.
        size_x: Default rectangle length in meters on the map X axis.
        size_y: Default rectangle width in meters on the map Y axis.
        color_rgba: Marker/debug color as normalized RGBA.
        aliases: Accepted detector labels or language-query aliases.
    """

    canonical_label: str
    display_label: str
    size_x: float
    size_y: float
    color_rgba: tuple[float, float, float, float]
    aliases: tuple[str, ...]


DEFAULT_SEMANTIC_CLASSES: dict[str, SemanticClass] = {
    "person": SemanticClass(
        canonical_label="person",
        display_label="人员",
        size_x=0.6,
        size_y=0.6,
        color_rgba=(1.0, 0.2, 0.2, 0.85),
        aliases=("person", "human", "people", "人", "人员"),
    ),
    "chair": SemanticClass(
        canonical_label="chair",
        display_label="椅子",
        size_x=0.6,
        size_y=0.6,
        color_rgba=(0.2, 0.6, 1.0, 0.85),
        aliases=("chair", "椅子", "轮椅", "seat"),
    ),
    "sofa": SemanticClass(
        canonical_label="sofa",
        display_label="沙发",
        size_x=1.8,
        size_y=0.9,
        color_rgba=(0.6, 0.2, 1.0, 0.85),
        aliases=("sofa", "couch", "沙发"),
    ),
    "bed": SemanticClass(
        canonical_label="bed",
        display_label="床",
        size_x=2.0,
        size_y=1.0,
        color_rgba=(0.2, 0.8, 0.8, 0.85),
        aliases=("bed", "床", "护理床"),
    ),
    "tea_table": SemanticClass(
        canonical_label="tea_table",
        display_label="茶几",
        size_x=1.0,
        size_y=0.6,
        color_rgba=(0.9, 0.6, 0.1, 0.85),
        aliases=("tea_table", "coffee table", "dining table", "table", "茶几", "桌子"),
    ),
    "toilet": SemanticClass(
        canonical_label="toilet",
        display_label="马桶",
        size_x=0.8,
        size_y=0.8,
        color_rgba=(0.1, 0.9, 0.3, 0.85),
        aliases=("toilet", "马桶", "坐便器", "卫生间", "厕所"),
    ),
    "cabinet": SemanticClass(
        canonical_label="cabinet",
        display_label="柜体",
        size_x=1.0,
        size_y=0.5,
        color_rgba=(0.7, 0.7, 0.7, 0.85),
        aliases=("cabinet", "cupboard", "柜体", "柜子"),
    ),
}


def build_alias_index(classes: Mapping[str, SemanticClass]) -> dict[str, SemanticClass]:
    """Build a case-insensitive lookup table for detector labels and aliases."""

    index: dict[str, SemanticClass] = {}
    for semantic_class in classes.values():
        index[semantic_class.canonical_label.lower()] = semantic_class
        for alias in semantic_class.aliases:
            index[alias.lower()] = semantic_class
    return index


def resolve_semantic_class(
    label: str,
    classes: Mapping[str, SemanticClass] | None = None,
) -> SemanticClass | None:
    """Resolve a detector or language label to a configured semantic class."""

    class_map = classes or DEFAULT_SEMANTIC_CLASSES
    return build_alias_index(class_map).get(label.strip().lower())
