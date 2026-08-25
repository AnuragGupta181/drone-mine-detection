#!/usr/bin/env python3
"""
path_corridor.py
================
Pure-function library: merges N scout paths into one human escape corridor.

Design:
  - Open/Closed Principle: add new merging strategies by subclassing
    CorridorMerger without touching the planner or swarm node.
  - Zero ROS imports — unit-testable standalone.
  - Zero side-effects: functions return new objects, never mutate inputs.

Typical usage:
    from path_corridor import CorridorMerger, ClearanceValidator

    merger = CorridorMerger(x_sample_step=1.0)
    human_path = merger.merge(scout_paths_world, mines, mine_clearance=1.0)

    ok, report = ClearanceValidator.validate(human_path, mines, 1.0)
"""

import math
from typing import List, Tuple, Optional, Dict


# ─────────────────────────────────────────────────────────────────────────────
# Type aliases (matches safe_path_planner conventions)
# ─────────────────────────────────────────────────────────────────────────────
Point2D   = Tuple[float, float]          # (world_x, world_y)
Path2D    = List[Point2D]                # ordered list of world-frame points
MineSpec  = Dict                         # {"position": [x, y, z], ...}


# ─────────────────────────────────────────────────────────────────────────────
# Internal geometry helpers
# ─────────────────────────────────────────────────────────────────────────────
def _dist(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _lerp_path_at_x(path: Path2D, query_x: float) -> Optional[float]:
    """
    Linearly interpolate the Y value of a path at a given X.
    Returns None if query_x is outside the path's X range.
    The path is assumed to be roughly monotonically increasing in X.
    """
    if not path:
        return None

    # Walk adjacent pairs
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        if min(x0, x1) <= query_x <= max(x0, x1):
            if abs(x1 - x0) < 1e-6:
                return (y0 + y1) / 2.0
            t = (query_x - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)

    # Extrapolation: clamp to endpoints
    if query_x <= path[0][0]:
        return path[0][1]
    return path[-1][1]


def _ramer_douglas_peucker(path: Path2D, tolerance: float = 0.3) -> Path2D:
    """Ramer–Douglas–Peucker polyline simplification (pure, no ROS)."""
    if len(path) <= 2:
        return list(path)

    def _pt_line_dist(p: Point2D, a: Point2D, b: Point2D) -> float:
        if a == b:
            return _dist(p, a)
        dx, dy = b[0] - a[0], b[1] - a[1]
        denom = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denom))
        proj = (a[0] + t * dx, a[1] + t * dy)
        return _dist(p, proj)

    max_d, idx = 0.0, 0
    for i in range(1, len(path) - 1):
        d = _pt_line_dist(path[i], path[0], path[-1])
        if d > max_d:
            max_d, idx = d, i

    if max_d > tolerance:
        left  = _ramer_douglas_peucker(path[:idx + 1], tolerance)
        right = _ramer_douglas_peucker(path[idx:],      tolerance)
        return left[:-1] + right
    return [path[0], path[-1]]


# ─────────────────────────────────────────────────────────────────────────────
# Strategy base class (Open/Closed Principle)
# ─────────────────────────────────────────────────────────────────────────────
class CorridorMergeStrategy:
    """Interface for corridor merge strategies. Override merge_paths()."""

    def merge_paths(
        self,
        paths: List[Path2D],
        x_start: float,
        x_end: float,
        x_step: float,
    ) -> Path2D:
        raise NotImplementedError


# ─────────────────────────────────────────────────────────────────────────────
# Concrete strategy: average-Y centre-line
# ─────────────────────────────────────────────────────────────────────────────
class AverageYStrategy(CorridorMergeStrategy):
    """
    At each X slice, average the Y values of all available lane paths.
    This produces a centre-line through the safe corridor bounded
    by the outermost scouted lanes.
    """

    def merge_paths(
        self,
        paths: List[Path2D],
        x_start: float,
        x_end: float,
        x_step: float,
    ) -> Path2D:
        merged: Path2D = []
        x = x_start
        while x <= x_end + 1e-6:
            ys = []
            for path in paths:
                y = _lerp_path_at_x(path, x)
                if y is not None:
                    ys.append(y)
            if ys:
                merged.append((x, sum(ys) / len(ys)))
            x += x_step

        return merged


# ─────────────────────────────────────────────────────────────────────────────
# Concrete strategy: maximum-clearance path (safest Y at each X)
# ─────────────────────────────────────────────────────────────────────────────
class MaxClearanceStrategy(CorridorMergeStrategy):
    """
    At each X slice, pick the Y from the available lane paths that
    is furthest from the nearest mine.  Falls back to AverageY if
    no mine data is provided.
    """

    def __init__(self, mines: List[MineSpec]):
        self._mines = mines

    def _clearance_at(self, wx: float, wy: float) -> float:
        if not self._mines:
            return float('inf')
        return min(
            math.hypot(wx - m['position'][0], wy - m['position'][1])
            for m in self._mines
        )

    def merge_paths(
        self,
        paths: List[Path2D],
        x_start: float,
        x_end: float,
        x_step: float,
    ) -> Path2D:
        merged: Path2D = []
        x = x_start
        while x <= x_end + 1e-6:
            candidates: List[Tuple[float, float]] = []  # (clearance, y)
            for path in paths:
                y = _lerp_path_at_x(path, x)
                if y is not None:
                    clr = self._clearance_at(x, y)
                    candidates.append((clr, y))
            if candidates:
                _, best_y = max(candidates, key=lambda c: c[0])
                merged.append((x, best_y))
            x += x_step
        return merged


# ─────────────────────────────────────────────────────────────────────────────
# Main merger class — context for the strategy pattern
# ─────────────────────────────────────────────────────────────────────────────
class CorridorMerger:
    """
    Merges N scout-lane paths into a single human escape path.

    Parameters
    ----------
    x_sample_step : float
        Spacing (metres) of X-samples when interpolating merged Y values.
    simplify_tolerance : float
        RDP simplification tolerance in metres (0 = no simplification).
    strategy : CorridorMergeStrategy | None
        Merge strategy.  Defaults to AverageYStrategy().
    """

    def __init__(
        self,
        x_sample_step: float = 0.5,
        simplify_tolerance: float = 0.3,
        strategy: Optional[CorridorMergeStrategy] = None,
    ):
        self._x_step      = x_sample_step
        self._rdp_tol     = simplify_tolerance
        self._strategy    = strategy or AverageYStrategy()

    def merge(
        self,
        scout_paths: List[Path2D],
        x_start: float,
        x_end: float,
    ) -> Path2D:
        """
        Returns the merged human-escape path (world frame, 2-D).
        Returns an empty list if no paths could be merged.
        """
        valid_paths = [p for p in scout_paths if p and len(p) >= 2]
        if not valid_paths:
            return []

        merged = self._strategy.merge_paths(
            valid_paths, x_start, x_end, self._x_step)

        if self._rdp_tol > 0 and len(merged) > 2:
            merged = _ramer_douglas_peucker(merged, self._rdp_tol)

        return merged


# ─────────────────────────────────────────────────────────────────────────────
# Clearance validator (used by verifier drone and by path merger)
# ─────────────────────────────────────────────────────────────────────────────
class ClearanceValidator:
    """
    Stateless validator: checks that every point in a path keeps at least
    `min_clearance_m` from every mine centre.
    """

    @staticmethod
    def validate(
        path: Path2D,
        mines: List[MineSpec],
        min_clearance_m: float,
    ) -> Tuple[bool, str]:
        """
        Returns (passed: bool, report: str).
        passed is True only if EVERY point in path clears ALL mines.
        """
        if not path:
            return False, "Path is empty — cannot validate."

        violations: List[str] = []
        for idx, (wx, wy) in enumerate(path):
            for mine in mines:
                mx, my = mine['position'][0], mine['position'][1]
                dist = math.hypot(wx - mx, wy - my)
                if dist < min_clearance_m:
                    violations.append(
                        f"WP{idx}=({wx:.2f},{wy:.2f}) is {dist:.2f}m from "
                        f"{mine.get('id','mine')} (need {min_clearance_m:.2f}m)"
                    )

        if violations:
            return False, "CLEARANCE FAILURES: " + "; ".join(violations)
        return True, f"All {len(path)} waypoints clear mines by ≥{min_clearance_m:.2f}m. ✓"

    @staticmethod
    def clearance_at(wx: float, wy: float, mines: List[MineSpec]) -> float:
        """Return minimum clearance from (wx,wy) to any mine centre."""
        if not mines:
            return float('inf')
        return min(
            math.hypot(wx - m['position'][0], wy - m['position'][1])
            for m in mines
        )
