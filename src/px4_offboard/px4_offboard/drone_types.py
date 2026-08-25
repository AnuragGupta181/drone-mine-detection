#!/usr/bin/env python3
"""
drone_types.py
==============
Shared data-only types for all drone state machines.
No ROS imports — importable without a running ROS context.

Design: Single Responsibility — this module ONLY defines types.
        All consumers import from here so changes propagate everywhere.
"""

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


# ─────────────────────────────────────────────────────────────────────────────
# Drone role taxonomy
# ─────────────────────────────────────────────────────────────────────────────
class DroneRole(Enum):
    SCOUT    = "SCOUT"     # Lane-scanning drone (instances 0-2)
    VERIFIER = "VERIFIER"  # Path-validation drone (instance 3)


# ─────────────────────────────────────────────────────────────────────────────
# Generic drone flight states shared across roles
# ─────────────────────────────────────────────────────────────────────────────
class FlightState(Enum):
    WARMUP   = auto()   # Waiting for DDS data + EKF2 readiness
    ARM      = auto()   # Arming + switching to OFFBOARD
    TAKEOFF  = auto()   # Climbing to cruise altitude
    NAVIGATE = auto()   # Flying through waypoints
    HOLD_MID = auto()   # Mid-field pause (scout only)
    HOLD_END = auto()   # End-of-mission hover before land
    LAND     = auto()   # Issuing land command
    DONE     = auto()   # Mission complete (terminal)

    # Verifier-specific states
    WAIT_SCOUTS  = auto()  # On ground, waiting until all scouts DONE
    VERIFY       = auto()  # Hovering at each human-path waypoint & checking
    REPORT       = auto()  # Publishing final verdict, then landing


# ─────────────────────────────────────────────────────────────────────────────
# Verification result per waypoint
# ─────────────────────────────────────────────────────────────────────────────
class WaypointVerdict(Enum):
    PENDING = "PENDING"
    PASS    = "PASS"
    FAIL    = "FAIL"


# ─────────────────────────────────────────────────────────────────────────────
# Lightweight waypoint type used throughout the stack
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Waypoint:
    """World-frame (Gazebo ENU) position in metres."""
    x: float
    y: float
    z: float = 0.0

    def distance_to(self, other: "Waypoint") -> float:
        import math
        return math.hypot(self.x - other.x, self.y - other.y)

    def to_tuple_2d(self) -> Tuple[float, float]:
        return (self.x, self.y)


# ─────────────────────────────────────────────────────────────────────────────
# Human escape path with verification state
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class HumanEscapePath:
    """
    The single merged safe corridor path for human guidance.
    Populated by PathCorridorMerger, verified by VerifierDroneController.
    """
    waypoints: List[Waypoint] = field(default_factory=list)
    verdicts:  List[WaypointVerdict] = field(default_factory=list)
    is_verified: bool = False
    overall_verdict: Optional[bool] = None   # True=SAFE, False=UNSAFE, None=pending

    def reset_verdicts(self) -> None:
        self.verdicts = [WaypointVerdict.PENDING] * len(self.waypoints)
        self.is_verified = False
        self.overall_verdict = None

    def record_verdict(self, wp_idx: int, passed: bool) -> None:
        if 0 <= wp_idx < len(self.verdicts):
            self.verdicts[wp_idx] = WaypointVerdict.PASS if passed else WaypointVerdict.FAIL

    def finalise(self) -> bool:
        """Compute and store overall verdict. Returns True if fully SAFE."""
        if not self.verdicts:
            self.overall_verdict = False
        else:
            self.overall_verdict = all(v == WaypointVerdict.PASS for v in self.verdicts)
        self.is_verified = True
        return bool(self.overall_verdict)


# ─────────────────────────────────────────────────────────────────────────────
# Per-drone configuration record
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class DroneConfig:
    """
    All configuration needed to build a DroneController.
    Decouples data (config) from behaviour (controller).
    """
    drone_id:   int
    namespace:  str            # ROS namespace, e.g. "" / "px4_1" / "px4_2" / "px4_3"
    role:       DroneRole
    waypoints:  List[Tuple[float, float]] = field(default_factory=list)
    mid_wp_idx: int = 0        # index of mid-field hold WP (SCOUT only)
    lane_y:     float = 0.0    # Gazebo-Y of scout lane (SCOUT only)
