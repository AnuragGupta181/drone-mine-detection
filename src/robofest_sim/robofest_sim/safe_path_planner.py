#!/usr/bin/env python3
"""
safe_path_planner.py
====================
Reads ground-truth mine and obstacle positions from the scenario manifest
(or from live /ground_truth/* topics), inflates a 2-D occupancy costmap
with 1 m mine-safety zones, runs A* for each of the 3 drone swarm lanes,
and publishes the results for RViz2 visualisation.

Published topics
----------------
/planning/safe_path/drone_0   nav_msgs/msg/Path
/planning/safe_path/drone_1   nav_msgs/msg/Path
/planning/safe_path/drone_2   nav_msgs/msg/Path
/planning/markers             visualization_msgs/msg/MarkerArray
    Includes:
      - mine exclusion circles  (red cylinders)
      - obstacle inflation zones (orange cylinders)
      - safe path lines         (green / cyan / yellow per drone)
      - start / exit zone boxes
      - drone waypoint spheres

Parameters
----------
manifest_path : str   Path to stage1_manifest.json (ground truth)
frame_id      : str   RViz fixed frame  (default "world")
grid_res      : float Costmap cell size  (default 0.25 m)
mine_radius   : float Mine physical radius (default 0.15 m)
mine_clearance: float Required clearance  (default 1.0 m)
drone_radius  : float Drone bounding radius (default 0.25 m)
sigma_safety  : float Extra uncertainty buffer (default 0.10 m)
flight_alt    : float Visualisation altitude (default 1.2 m)
publish_rate  : float Hz  (default 2.0)
"""

import json
import math
import os
import heapq
from typing import List, Tuple, Optional
from builtin_interfaces.msg import Duration

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String, Bool

# Internal pure-function library (no ROS dep)
from robofest_sim.path_corridor import (
    CorridorMerger, MaxClearanceStrategy, ClearanceValidator
)


# ─────────────────────────────────────────────────────────────────────────────
# Colour helpers
# ─────────────────────────────────────────────────────────────────────────────
def rgba(r, g, b, a=1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


DRONE_COLORS = [
    rgba(0.0, 1.0, 0.0),   # Drone 0 → green
    rgba(0.0, 0.8, 1.0),   # Drone 1 → cyan
    rgba(1.0, 0.9, 0.0),   # Drone 2 → yellow
]

# Y-lane centres for 3 drones in the world frame (Gazebo)
DRONE_LANE_Y = [-2.0, 0.0, 2.0]


# ─────────────────────────────────────────────────────────────────────────────
# 2-D A* path planner on an occupancy grid
# ─────────────────────────────────────────────────────────────────────────────
class AStarPlanner:
    """A* search on a 2-D boolean occupancy grid."""

    def __init__(self, grid, resolution: float, x_offset: float, y_offset: float):
        """
        grid      : list[list[bool]]  grid[ix][iy] = True if FREE
        resolution: metres per cell
        x_offset  : world-X of grid column 0
        y_offset  : world-Y of grid row 0
        """
        self.grid = grid
        self.res = resolution
        self.x_off = x_offset
        self.y_off = y_offset
        self.nx = len(grid)
        self.ny = len(grid[0]) if self.nx > 0 else 0

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        ix = int((wx - self.x_off) / self.res)
        iy = int((wy - self.y_off) / self.res)
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        wx = self.x_off + (ix + 0.5) * self.res
        wy = self.y_off + (iy + 0.5) * self.res
        return wx, wy

    def _heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan(self, start_world, goal_world) -> Optional[List[Tuple[float, float]]]:
        """Return list of (wx, wy) world-frame waypoints, or None if no path."""
        sx, sy = self.world_to_grid(*start_world)
        gx, gy = self.world_to_grid(*goal_world)

        # Clamp to grid bounds
        sx = max(0, min(sx, self.nx - 1))
        sy = max(0, min(sy, self.ny - 1))
        gx = max(0, min(gx, self.nx - 1))
        gy = max(0, min(gy, self.ny - 1))

        if not self.grid[sx][sy]:
            # Nudge start to nearest free cell (5-cell search)
            found = False
            for r in range(1, 6):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx2, ny2 = sx + dx, sy + dy
                        if 0 <= nx2 < self.nx and 0 <= ny2 < self.ny and self.grid[nx2][ny2]:
                            sx, sy = nx2, ny2
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                return None

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}
        DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (gx, gy):
                # Reconstruct
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(*current))
                    current = came_from[current]
                path.append(self.grid_to_world(*current))
                path.reverse()
                return path

            for dx, dy in DIRS:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < self.nx and 0 <= nxt[1] < self.ny):
                    continue
                if not self.grid[nxt[0]][nxt[1]]:
                    continue
                step = math.hypot(dx, dy) * self.res
                tentative_g = g_score[current] + step
                if tentative_g < g_score.get(nxt, float('inf')):
                    came_from[nxt] = current
                    g_score[nxt] = tentative_g
                    f = tentative_g + self._heuristic(nxt, (gx, gy)) * self.res
                    heapq.heappush(open_set, (f, nxt))

        return None  # No path found


def path_simplify(path: List[Tuple[float, float]],
                  tolerance: float = 0.5) -> List[Tuple[float, float]]:
    """Ramer-Douglas-Peucker line simplification."""
    if len(path) <= 2:
        return path

    def point_line_dist(p, a, b):
        if a == b:
            return math.hypot(p[0] - a[0], p[1] - a[1])
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        proj = (a[0] + t * dx, a[1] + t * dy)
        return math.hypot(p[0] - proj[0], p[1] - proj[1])

    max_dist = 0.0
    index = 0
    for i in range(1, len(path) - 1):
        d = point_line_dist(path[i], path[0], path[-1])
        if d > max_dist:
            max_dist = d
            index = i

    if max_dist > tolerance:
        left = path_simplify(path[:index + 1], tolerance)
        right = path_simplify(path[index:], tolerance)
        return left[:-1] + right
    else:
        return [path[0], path[-1]]


# ─────────────────────────────────────────────────────────────────────────────
# Main ROS 2 Node
# ─────────────────────────────────────────────────────────────────────────────
class SafePathPlannerNode(Node):

    def __init__(self):
        super().__init__('safe_path_planner')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('manifest_path',
                               '/home/ubuntu/px4_ros2_ws/src/robofest_sim/worlds/generated/stage1_manifest.json')
        self.declare_parameter('frame_id',     'world')
        self.declare_parameter('grid_res',     0.25)
        self.declare_parameter('mine_radius',  0.15)
        self.declare_parameter('mine_clearance', 1.0)
        self.declare_parameter('drone_radius', 0.25)
        self.declare_parameter('sigma_safety', 0.10)
        self.declare_parameter('flight_alt',   1.2)
        self.declare_parameter('publish_rate', 5.0)

        self.manifest_path   = self.get_parameter('manifest_path').value
        self.frame_id        = self.get_parameter('frame_id').value
        self.grid_res        = self.get_parameter('grid_res').value
        self.mine_radius     = self.get_parameter('mine_radius').value
        self.mine_clearance  = self.get_parameter('mine_clearance').value
        self.drone_radius    = self.get_parameter('drone_radius').value
        self.sigma_safety    = self.get_parameter('sigma_safety').value
        self.flight_alt      = self.get_parameter('flight_alt').value
        self.publish_rate    = self.get_parameter('publish_rate').value

        # Inflation radius (from architecture formula):
        # R_inflation = r_mine + 1.0m + r_drone + sigma_safety
        self.R_inflation = (self.mine_radius + self.mine_clearance
                            + self.drone_radius + self.sigma_safety)
        self.get_logger().info(
            f"Mine exclusion radius R_inflation = {self.R_inflation:.2f} m "
            f"(mine={self.mine_radius:.2f} + 1m + drone={self.drone_radius:.2f}"
            f" + sigma={self.sigma_safety:.2f})")

        # ── Publishers ────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.path_pubs = [
            self.create_publisher(Path, f'/planning/safe_path/drone_{i}', qos)
            for i in range(3)
        ]
        self.marker_pub     = self.create_publisher(MarkerArray, '/planning/markers', qos)
        # Human escape path: merged single path for the verifier drone + human guidance
        self.human_path_pub = self.create_publisher(Path, '/planning/human_path', qos)
        # Verification result subscriber (published by verifier drone node)
        self.verdict_sub    = self.create_subscription(
            String, '/mission/verification_report', self._verdict_cb, 10)
        self._latest_verdict: Optional[str] = None

        # Subscribe to scouts_done to reveal minefield + human path
        # Must match publisher QoS (RELIABLE + TRANSIENT_LOCAL)
        _reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scouts_done_sub = self.create_subscription(
            Bool, '/mission/scouts_done', self._scouts_done_cb, _reliable_qos)
        self.show_mines = False    # True once >=2 scouts land: reveal mines & drone paths
        self.show_paths = False    # same gate — drone paths shown at the same time

        # ── Internal state ───────────────────────────────────────────────────
        self.manifest        = None
        self.mines           = []
        self.obstacles       = []
        self.field_length    = 40.0
        self.field_width     = 10.0
        self.start_zone      = {'x_min': 0,  'x_max': 5,  'y_min': -5, 'y_max': 5}
        self.exit_zone       = {'x_min': 35, 'x_max': 40, 'y_min': -5, 'y_max': 5}
        self.planned_paths   = [None, None, None]  # world-frame list of (x,y) per scout
        self.human_path      : List[Tuple[float, float]] = []  # merged corridor

        # ── Animation state (for pulsing mine danger rings) ──────────────────
        self._ring_phase: float = 0.0   # 0.0 → 2π, advances each publish tick

        # ── Timer ────────────────────────────────────────────────────────────
        self._plan_timer = self.create_timer(
            1.0 / self.publish_rate, self._timer_cb)

        self.get_logger().info('SafePathPlannerNode started. Waiting for manifest...')

    # ─────────────────────────────────────────────────────────────────────────
    def _load_manifest(self) -> bool:
        if not os.path.exists(self.manifest_path):
            return False
        try:
            with open(self.manifest_path, 'r') as f:
                self.manifest = json.load(f)
            self.mines     = self.manifest.get('mines', [])
            self.obstacles = self.manifest.get('obstacles', [])
            fl = self.manifest.get('field', {})
            self.field_length = fl.get('length', 40.0)
            self.field_width  = fl.get('width',  10.0)
            zones = self.manifest.get('zones', {})
            if 'start_zone' in zones:
                self.start_zone = zones['start_zone']
            if 'exit_zone' in zones:
                self.exit_zone = zones['exit_zone']
            self.get_logger().info(
                f"Manifest loaded: {len(self.mines)} mines, "
                f"{len(self.obstacles)} obstacles")
            return True
        except Exception as e:
            self.get_logger().warn(f'Failed to load manifest: {e}')
            return False

    # ─────────────────────────────────────────────────────────────────────────
    def _build_costmap(self, lane_y_centre: float):
        """
        Build a 2-D boolean free/occupied grid.
        lane_y_centre constrains the Y-band each drone searches in
        (±2 m around the lane). Returns (grid, x_off, y_off).
        """
        res    = self.grid_res
        x_off  = 0.0
        y_off  = -self.field_width / 2.0
        nx     = int(self.field_length / res) + 1
        ny     = int(self.field_width  / res) + 1

        # All cells start free
        grid = [[True] * ny for _ in range(nx)]

        # Mark field-boundary margin as occupied (0.3 m from edge)
        margin_cells = int(0.3 / res)
        for ix in range(nx):
            for m in range(margin_cells):
                if m < ny:
                    grid[ix][m] = False
                if ny - 1 - m >= 0:
                    grid[ix][ny - 1 - m] = False

        # Mark mine exclusion zones
        for mine in self.mines:
            mx, my = mine['position'][0], mine['position'][1]
            for ix in range(nx):
                wx = x_off + (ix + 0.5) * res
                for iy in range(ny):
                    wy = y_off + (iy + 0.5) * res
                    if math.hypot(wx - mx, wy - my) <= self.R_inflation:
                        grid[ix][iy] = False

        # Mark obstacle inflation zones (obstacle_radius + drone_radius + 0.2m buffer)
        obs_inflate = self.drone_radius + 0.5  # 0.5 m obstacle clearance
        for obs in self.obstacles:
            ox, oy = obs['position'][0], obs['position'][1]
            for ix in range(nx):
                wx = x_off + (ix + 0.5) * res
                for iy in range(ny):
                    wy = y_off + (iy + 0.5) * res
                    if math.hypot(wx - ox, wy - oy) <= (0.4 + obs_inflate):
                        grid[ix][iy] = False

        # Mark the two static trees from the scenario (Gazebo: 18,2 and 23,-2)
        for tx, ty in [(18.0, 2.0), (23.0, -2.0)]:
            tree_r = 0.8  # trunk + safety
            for ix in range(nx):
                wx = x_off + (ix + 0.5) * res
                for iy in range(ny):
                    wy = y_off + (iy + 0.5) * res
                    if math.hypot(wx - tx, wy - ty) <= tree_r:
                        grid[ix][iy] = False

        return grid, nx, ny, x_off, y_off

    # ─────────────────────────────────────────────────────────────────────────
    def _plan_all_paths(self):
        """Run A* for each of the 3 drone lanes, then merge into human corridor."""
        self.planned_paths = [None, None, None]

        for drone_id, lane_y in enumerate(DRONE_LANE_Y):
            grid, nx, ny, x_off, y_off = self._build_costmap(lane_y)
            planner = AStarPlanner(grid, self.grid_res, x_off, y_off)

            # Start: middle of start zone, at lane Y
            start_x = (self.start_zone['x_min'] + self.start_zone['x_max']) / 2.0
            # Exit: middle of exit zone, at lane Y
            goal_x  = (self.exit_zone['x_min']  + self.exit_zone['x_max'])  / 2.0

            # Clamp Y to field bounds
            lane_y_clamped = max(-self.field_width / 2.0 + 0.5,
                                 min(self.field_width / 2.0 - 0.5, lane_y))

            raw_path = planner.plan(
                (start_x, lane_y_clamped),
                (goal_x,  lane_y_clamped))

            if raw_path is None:
                self.get_logger().warn(
                    f'Drone {drone_id}: A* found NO path for lane Y={lane_y:.1f}m! '
                    f'Trying centre lane...')
                raw_path = planner.plan((start_x, 0.0), (goal_x, 0.0))

            if raw_path is not None:
                simplified = path_simplify(raw_path, tolerance=0.4)
                self.planned_paths[drone_id] = simplified
                self.get_logger().info(
                    f'Drone {drone_id}: path planned — {len(simplified)} waypoints '
                    f'({len(raw_path)} raw → simplified)')
            else:
                self.get_logger().error(
                    f'Drone {drone_id}: A* FAILED — no navigable corridor exists!')

        # ── Merge all scout paths into a single human corridor ────────────────
        self._merge_human_path()

    # ─────────────────────────────────────────────────────────────────────────
    def _merge_human_path(self) -> None:
        """
        Merge 3 scout paths → one human escape corridor using
        MaxClearanceStrategy (prefers path segments furthest from mines).
        Validates the merged corridor and logs the result.
        """
        valid_paths = [p for p in self.planned_paths if p]
        if not valid_paths:
            self.get_logger().error('No scout paths available — cannot compute human path!')
            return

        x_start = (self.start_zone['x_min'] + self.start_zone['x_max']) / 2.0
        x_end   = (self.exit_zone['x_min']  + self.exit_zone['x_max'])  / 2.0

        # Use MaxClearance strategy so the human always walks the widest gap from mines
        merger = CorridorMerger(
            x_sample_step=0.5,
            simplify_tolerance=0.3,
            strategy=MaxClearanceStrategy(mines=self.mines),
        )
        self.human_path = merger.merge(valid_paths, x_start, x_end)

        if not self.human_path:
            self.get_logger().error('Corridor merger returned empty path!')
            return

        # Validate merged path against mine clearances
        passed, report = ClearanceValidator.validate(
            self.human_path, self.mines, self.mine_clearance)
        level = self.get_logger().info if passed else self.get_logger().warn
        level(f'[HumanPath] {report}')
        self.get_logger().info(
            f'[HumanPath] Merged corridor: {len(self.human_path)} waypoints '
            f'(from {len(valid_paths)} scout lanes)')

    # ─────────────────────────────────────────────────────────────────────────
    def _scouts_done_cb(self, msg: Bool) -> None:
        """Trigger to reveal the hidden minefield, drone paths and human path."""
        if msg.data and not self.show_mines:
            self.get_logger().info('At least 2 scouts landed — revealing minefield, safe paths, and human path in RViz.')
            self.show_mines = True
            self.show_paths = True

    # ─────────────────────────────────────────────────────────────────────────
    def _verdict_cb(self, msg: String) -> None:
        """Receive verification report from the verifier drone."""
        self._latest_verdict = msg.data
        self.get_logger().info(f'[Verifier Report] {msg.data}')

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_paths(self):
        now = self.get_clock().now().to_msg()
        for drone_id, path_pts in enumerate(self.planned_paths):
            msg = Path()
            msg.header.stamp    = now
            msg.header.frame_id = self.frame_id
            if path_pts is not None:
                for (wx, wy) in path_pts:
                    ps = PoseStamped()
                    ps.header.stamp    = now
                    ps.header.frame_id = self.frame_id
                    ps.pose.position.x = wx
                    ps.pose.position.y = wy
                    ps.pose.position.z = self.flight_alt
                    ps.pose.orientation.w = 1.0
                    msg.poses.append(ps)
            self.path_pubs[drone_id].publish(msg)

        # Publish merged human escape path (only after 2 scouts land)
        if self.human_path and getattr(self, 'show_mines', False):
            hmsg = Path()
            hmsg.header.stamp    = now
            hmsg.header.frame_id = self.frame_id
            for (wx, wy) in self.human_path:
                ps = PoseStamped()
                ps.header.stamp    = now
                ps.header.frame_id = self.frame_id
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ps.pose.position.z = 0.1   # ground-level for human
                ps.pose.orientation.w = 1.0
                hmsg.poses.append(ps)
            self.human_path_pub.publish(hmsg)

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_markers(self):
        now  = self.get_clock().now().to_msg()
        ma   = MarkerArray()
        mid  = 0   # sequential marker ID

        lifetime = Duration()
        lifetime.sec = 1  # markers refresh every publish cycle

        # ── Mine danger zone (ALWAYS VISIBLE, filled low-opacity red circle) ──
        for mine in self.mines:
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp    = now
            m.ns              = 'mine_danger_ring'
            m.id              = mid; mid += 1
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.lifetime        = lifetime
            m.pose.position.x = float(mine['position'][0])
            m.pose.position.y = float(mine['position'][1])
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = self.R_inflation * 2.0
            m.scale.y = self.R_inflation * 2.0
            m.scale.z = 0.05
            m.color   = rgba(1.0, 0.0, 0.0, 0.15)   # transparent low red
            ma.markers.append(m)

        # ── Mine body (solid red disc, HIDDEN UNTIL 2 SCOUTS LAND) ──────────
        if getattr(self, 'show_mines', False):
            for mine in self.mines:
                # Mine body (solid red disc)
                m2 = Marker()
                m2.header.frame_id = self.frame_id
                m2.header.stamp    = now
                m2.ns              = 'mine_body'
                m2.id              = mid; mid += 1
                m2.type            = Marker.CYLINDER
                m2.action          = Marker.ADD
                m2.lifetime        = lifetime
                m2.pose.position.x = float(mine['position'][0])
                m2.pose.position.y = float(mine['position'][1])
                m2.pose.position.z = 0.05
                m2.pose.orientation.w = 1.0
                m2.scale.x = self.mine_radius * 2.0
                m2.scale.y = self.mine_radius * 2.0
                m2.scale.z = 0.12
                m2.color   = rgba(1.0, 0.0, 0.0, 0.9)
                ma.markers.append(m2)

            # ── Obstacle zones ────────────────────────────────────────────────────
            for obs in self.obstacles:
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp    = now
                m.ns              = 'obstacle_zone'
                m.id              = mid; mid += 1
                m.type            = Marker.CYLINDER
                m.action          = Marker.ADD
                m.lifetime        = lifetime
                m.pose.position.x = float(obs['position'][0])
                m.pose.position.y = float(obs['position'][1])
                m.pose.position.z = 0.5
                m.pose.orientation.w = 1.0
                m.scale.x = 1.0
                m.scale.y = 1.0
                m.scale.z = 1.5
                m.color   = rgba(1.0, 0.55, 0.0, 0.7)   # orange
                ma.markers.append(m)

        # ── Start zone box (ALWAYS VISIBLE) ───────────────────────────────────
        sz = self.start_zone
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = now
        m.ns              = 'start_zone'
        m.id              = mid; mid += 1
        m.type            = Marker.CUBE
        m.action          = Marker.ADD
        m.lifetime        = lifetime
        m.pose.position.x = (sz['x_min'] + sz['x_max']) / 2.0
        m.pose.position.y = (sz['y_min'] + sz['y_max']) / 2.0
        m.pose.position.z = 0.02
        m.pose.orientation.w = 1.0
        m.scale.x = sz['x_max'] - sz['x_min']
        m.scale.y = sz['y_max'] - sz['y_min']
        m.scale.z = 0.04
        m.color   = rgba(0.0, 1.0, 0.2, 0.35)   # green
        ma.markers.append(m)

        # START text
        mt = Marker()
        mt.header.frame_id = self.frame_id
        mt.header.stamp    = now
        mt.ns              = 'zone_labels'
        mt.id              = mid; mid += 1
        mt.type            = Marker.TEXT_VIEW_FACING
        mt.action          = Marker.ADD
        mt.lifetime        = lifetime
        mt.pose.position.x = (sz['x_min'] + sz['x_max']) / 2.0
        mt.pose.position.y = sz['y_min'] - 0.5
        mt.pose.position.z = 0.5
        mt.pose.orientation.w = 1.0
        mt.scale.z         = 0.8
        mt.color           = rgba(0.0, 1.0, 0.2)
        mt.text            = 'START ZONE'
        ma.markers.append(mt)

        # ── Exit zone box ─────────────────────────────────────────────────────
        ez = self.exit_zone
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = now
        m.ns              = 'exit_zone'
        m.id              = mid; mid += 1
        m.type            = Marker.CUBE
        m.action          = Marker.ADD
        m.lifetime        = lifetime
        m.pose.position.x = (ez['x_min'] + ez['x_max']) / 2.0
        m.pose.position.y = (ez['y_min'] + ez['y_max']) / 2.0
        m.pose.position.z = 0.02
        m.pose.orientation.w = 1.0
        m.scale.x = ez['x_max'] - ez['x_min']
        m.scale.y = ez['y_max'] - ez['y_min']
        m.scale.z = 0.04
        m.color   = rgba(0.0, 0.5, 1.0, 0.35)   # blue
        ma.markers.append(m)

        # EXIT text
        mt = Marker()
        mt.header.frame_id = self.frame_id
        mt.header.stamp    = now
        mt.ns              = 'zone_labels'
        mt.id              = mid; mid += 1
        mt.type            = Marker.TEXT_VIEW_FACING
        mt.action          = Marker.ADD
        mt.lifetime        = lifetime
        mt.pose.position.x = (ez['x_min'] + ez['x_max']) / 2.0
        mt.pose.position.y = ez['y_min'] - 0.5
        mt.pose.position.z = 0.5
        mt.pose.orientation.w = 1.0
        mt.scale.z         = 0.8
        mt.color           = rgba(0.0, 0.5, 1.0)
        mt.text            = 'EXIT ZONE'
        ma.markers.append(mt)

        # ── Safe paths (LINE_STRIP per drone, HIDDEN until 2 scouts land) ──────
        if getattr(self, 'show_paths', False):
            for drone_id, path_pts in enumerate(self.planned_paths):
                if path_pts is None:
                    continue
                color = DRONE_COLORS[drone_id]

                # Path line
                m = Marker()
                m.header.frame_id = self.frame_id
                m.header.stamp    = now
                m.ns              = f'safe_path_{drone_id}'
                m.id              = mid; mid += 1
                m.type            = Marker.LINE_STRIP
                m.action          = Marker.ADD
                m.lifetime        = lifetime
                m.scale.x         = 0.12   # line width
                m.color           = color
                m.pose.orientation.w = 1.0
                for (wx, wy) in path_pts:
                    p = Point()
                    p.x = wx; p.y = wy; p.z = self.flight_alt
                    m.points.append(p)
                ma.markers.append(m)

                # Waypoint spheres
                for wp_idx, (wx, wy) in enumerate(path_pts):
                    ms = Marker()
                    ms.header.frame_id = self.frame_id
                    ms.header.stamp    = now
                    ms.ns              = f'path_waypoints_{drone_id}'
                    ms.id              = mid; mid += 1
                    ms.type            = Marker.SPHERE
                    ms.action          = Marker.ADD
                    ms.lifetime        = lifetime
                    ms.pose.position.x = wx
                    ms.pose.position.y = wy
                    ms.pose.position.z = self.flight_alt
                    ms.pose.orientation.w = 1.0
                    ms.scale.x = ms.scale.y = ms.scale.z = 0.25
                    ms.color   = color
                    ma.markers.append(ms)

                # Drone label
                if path_pts:
                    ml = Marker()
                    ml.header.frame_id = self.frame_id
                    ml.header.stamp    = now
                    ml.ns              = 'drone_labels'
                    ml.id              = mid; mid += 1
                    ml.type            = Marker.TEXT_VIEW_FACING
                    ml.action          = Marker.ADD
                    ml.lifetime        = lifetime
                    ml.pose.position.x = path_pts[0][0]
                    ml.pose.position.y = path_pts[0][1]
                    ml.pose.position.z = self.flight_alt + 0.5
                    ml.pose.orientation.w = 1.0
                    ml.scale.z         = 0.5
                    ml.color           = color
                    ml.text            = f'Drone {drone_id}'
                    ma.markers.append(ml)

        # ── Human escape corridor (merged) ────────────────────────────────────
        if self.human_path and getattr(self, 'show_mines', False):
            # Determine colour based on latest verification verdict
            if self._latest_verdict is None:
                hcolor = rgba(1.0, 1.0, 1.0, 0.9)          # white  = not yet verified
                label_text = 'AWAITING VERIFICATION'
            elif 'SAFE' in (self._latest_verdict or '').upper():
                hcolor = rgba(0.0, 1.0, 0.4, 1.0)           # bright green = SAFE
                label_text = '✓ SAFE — HUMAN MAY PROCEED'
            else:
                hcolor = rgba(1.0, 0.2, 0.0, 1.0)           # red = UNSAFE
                label_text = '✗ UNSAFE — RE-PLANNING NEEDED'

            # Thick ground-level LINE_STRIP
            hm = Marker()
            hm.header.frame_id = self.frame_id
            hm.header.stamp    = now
            hm.ns              = 'human_path'
            hm.id              = mid; mid += 1
            hm.type            = Marker.LINE_STRIP
            hm.action          = Marker.ADD
            hm.lifetime        = lifetime
            hm.scale.x         = 0.22    # wider than drone paths
            hm.color           = hcolor
            hm.pose.orientation.w = 1.0
            for (wx, wy) in self.human_path:
                p = Point(); p.x = wx; p.y = wy; p.z = 0.15
                hm.points.append(p)
            ma.markers.append(hm)

            # Diamond (sphere) at each human waypoint
            for (wx, wy) in self.human_path:
                hd = Marker()
                hd.header.frame_id = self.frame_id
                hd.header.stamp    = now
                hd.ns              = 'human_waypoints'
                hd.id              = mid; mid += 1
                hd.type            = Marker.SPHERE
                hd.action          = Marker.ADD
                hd.lifetime        = lifetime
                hd.pose.position.x = wx
                hd.pose.position.y = wy
                hd.pose.position.z = 0.2
                hd.pose.orientation.w = 1.0
                hd.scale.x = hd.scale.y = hd.scale.z = 0.35
                hd.color   = hcolor
                ma.markers.append(hd)

            # Floating verdict label above mid-field
            mid_wp = self.human_path[len(self.human_path) // 2]
            hl = Marker()
            hl.header.frame_id = self.frame_id
            hl.header.stamp    = now
            hl.ns              = 'human_verdict'
            hl.id              = mid; mid += 1
            hl.type            = Marker.TEXT_VIEW_FACING
            hl.action          = Marker.ADD
            hl.lifetime        = lifetime
            hl.pose.position.x = mid_wp[0]
            hl.pose.position.y = mid_wp[1]
            hl.pose.position.z = 2.5
            hl.pose.orientation.w = 1.0
            hl.scale.z         = 0.7
            hl.color           = hcolor
            hl.text            = label_text
            ma.markers.append(hl)

        self.marker_pub.publish(ma)

    # ─────────────────────────────────────────────────────────────────────────
    def _timer_cb(self):
        # Load manifest if not yet loaded
        if self.manifest is None:
            if not self._load_manifest():
                return  # wait silently
            # Plan paths once after first manifest load
            self._plan_all_paths()

        self._publish_paths()
        self._publish_markers()


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SafePathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
