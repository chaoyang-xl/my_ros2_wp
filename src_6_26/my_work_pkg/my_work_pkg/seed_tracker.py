#!/usr/bin/env python3
"""轻量级语义种子追踪器：EMA 平滑 + 置信度累积 + 生命周期管理。

不依赖占用栅格地图，直接在 map 坐标系下对 /semantic_seed 做追踪。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot
from time import monotonic


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class TrackedSeed:
    """一个被追踪的语义种子对象。"""

    object_id: str         # label_0, label_1, ...
    label: str             # 原始 label（小写、下划线）
    x: float               # EMA 平滑后的 x (map 系，米)
    y: float               # EMA 平滑后的 y (map 系，米)
    confidence: float       # 历史平均置信度
    times_seen: int         # 累计被观测次数
    last_seen: float        # 最后一次观测时间 (monotonic seconds)
    created_at: float       # 首次创建时间 (monotonic seconds)


# ---------------------------------------------------------------------------
# Tracker
# ---------------------------------------------------------------------------

class SeedTracker:
    """语义种子追踪器：匹配 → EMA 平滑 → 老化淘汰。"""

    def __init__(
        self,
        match_distance: float = 1.0,
        alpha: float = 0.3,
        timeout: float = 10.0,
    ) -> None:
        """
        Args:
            match_distance: 同名对象匹配的欧氏距离阈值 (米)。
            alpha:          EMA 平滑系数。0=不动, 0.5=各半, 1=完全信任新值。
            timeout:        超时秒数，超过此时间未被观测的对象将被移除。
        """
        if match_distance <= 0:
            raise ValueError("match_distance must be positive")
        if not 0 < alpha <= 1:
            raise ValueError("alpha must be in (0, 1]")
        if timeout <= 0:
            raise ValueError("timeout must be positive")

        self.match_distance = match_distance
        self.alpha = alpha
        self.timeout = timeout

        self._objects: dict[str, TrackedSeed] = {}
        self._next_id: dict[str, int] = {}

    # ---- public API ----

    def update(self, label: str, confidence: float, gx: float, gy: float) -> TrackedSeed:
        """接收一个新观测，匹配或创建，返回更新后的 TrackedSeed。

        Args:
            label:      物体类别 (e.g. ``"chair"``)。
            confidence: 本次观测的置信度 (0~1)。
            gx, gy:     本次观测在 map 坐标系下的坐标 (米)。

        Returns:
            更新后（或新建）的追踪对象。
        """
        now = monotonic()
        label = _norm_label(label)

        # 1. 在同 label 中找到最近的匹配
        best = None
        best_dist = float("inf")
        for obj in self._objects.values():
            if obj.label != label:
                continue
            d = hypot(obj.x - gx, obj.y - gy)
            if d < self.match_distance and d < best_dist:
                best = obj
                best_dist = d

        if best is not None:
            # 2a. EMA 平滑
            a = self.alpha
            best.x = a * gx + (1 - a) * best.x
            best.y = a * gy + (1 - a) * best.y
            # 置信度：历史平均
            best.confidence = (
                (best.confidence * best.times_seen + confidence)
                / (best.times_seen + 1)
            )
            best.times_seen += 1
            best.last_seen = now
            return best

        # 2b. 新建
        seq = self._next_id.get(label, 0)
        self._next_id[label] = seq + 1
        oid = f"{label}_{seq}"
        obj = TrackedSeed(
            object_id=oid,
            label=label,
            x=gx,
            y=gy,
            confidence=confidence,
            times_seen=1,
            last_seen=now,
            created_at=now,
        )
        self._objects[oid] = obj
        return obj

    def age(self) -> list[TrackedSeed]:
        """移除超时对象，返回被移除的列表（可选用于日志）。"""
        now = monotonic()
        stale_oids = [
            oid
            for oid, obj in self._objects.items()
            if now - obj.last_seen > self.timeout
        ]
        removed = [self._objects[oid] for oid in stale_oids]
        for oid in stale_oids:
            del self._objects[oid]
        return removed

    def get_active(self) -> list[TrackedSeed]:
        """返回当前所有活跃的追踪对象 (按创建时间排序)。"""
        return sorted(self._objects.values(), key=lambda o: o.created_at)

    def clear(self) -> None:
        """清空所有追踪对象。"""
        self._objects.clear()
        self._next_id.clear()

    def __len__(self) -> int:
        return len(self._objects)


def _norm_label(label: str) -> str:
    return (label.strip() or "unknown").lower().replace(" ", "_")
