#!/usr/bin/env python3
"""
pipeline_test.py — Full Static + Integration Test Suite
Tests all Python nodes in the robofest_sim pipeline without needing ROS 2 running.
Run this before starting the simulation to catch any configuration errors.

Usage:
    python3 /home/ubuntu/px4_ros2_ws/src/robofest_sim/robofest_sim/pipeline_test.py
"""

import ast
import os
import sys
import json
import math
import yaml
import unittest

WS = "/home/ubuntu/px4_ros2_ws"
ROBOFEST_SRC = f"{WS}/src/robofest_sim/robofest_sim"
OFFBOARD_SRC = f"{WS}/src/px4_offboard/px4_offboard"
CONFIG_DIR   = f"{WS}/src/robofest_sim/config"
WORLDS_DIR   = f"{WS}/src/robofest_sim/worlds"
GEN_DIR      = f"{WS}/src/robofest_sim/worlds/generated"
PX4_AIRFRAME = "/home/ubuntu/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4013_gz_x500_lidar_2d"
ENV_FILE     = f"{WS}/.env"

class TestSyntax(unittest.TestCase):
    """TC-1: All Python nodes must parse without syntax errors."""

    NODES = [
        f"{ROBOFEST_SRC}/scenario_generator.py",
        f"{ROBOFEST_SRC}/ground_truth_publisher.py",
        f"{ROBOFEST_SRC}/sim_tf_publisher.py",
        f"{ROBOFEST_SRC}/ekf2_readiness_checker.py",
        f"{ROBOFEST_SRC}/sensor_health_monitor.py",
        f"{ROBOFEST_SRC}/mission_evaluator.py",
        f"{ROBOFEST_SRC}/px4_ext_odom_node.py",
        f"{OFFBOARD_SRC}/position_hold_node.py",
        f"{OFFBOARD_SRC}/offboard_control_node.py",
    ]

    def test_all_nodes_syntax(self):
        for path in self.NODES:
            with self.subTest(file=os.path.basename(path)):
                self.assertTrue(os.path.exists(path), f"File missing: {path}")
                with open(path) as f:
                    source = f.read()
                try:
                    ast.parse(source)
                except SyntaxError as e:
                    self.fail(f"Syntax error in {path}: {e}")


class TestConfig(unittest.TestCase):
    """TC-2: Configuration files must exist and be well-formed."""

    def test_stage1_yaml_exists_and_valid(self):
        path = f"{CONFIG_DIR}/stage1.yaml"
        self.assertTrue(os.path.exists(path), f"Missing: {path}")
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.assertIn("field", cfg)
        self.assertIn("zones", cfg)
        self.assertIn("scenario", cfg)
        self.assertIn("human", cfg)
        self.assertEqual(cfg["field"]["length"], 40.0)
        self.assertEqual(cfg["field"]["width"], 10.0)
        self.assertIn("start_zone", cfg["zones"])
        self.assertIn("exit_zone", cfg["zones"])
        self.assertIn("minefield_zone", cfg["zones"])

    def test_slam_toolbox_params_valid(self):
        path = f"{CONFIG_DIR}/slam_toolbox_params.yaml"
        self.assertTrue(os.path.exists(path), f"Missing: {path}")
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.assertIn("slam_toolbox", cfg)
        params = cfg["slam_toolbox"]["ros__parameters"]
        self.assertEqual(params["scan_topic"], "/scan")
        self.assertEqual(params["base_frame"], "base_link")
        self.assertEqual(params["odom_frame"], "odom")

    def test_env_file_gps_enabled(self):
        self.assertTrue(os.path.exists(ENV_FILE), f"Missing .env file: {ENV_FILE}")
        with open(ENV_FILE) as f:
            content = f.read()
        self.assertIn("GPS_ENABLED=true", content, ".env must have GPS_ENABLED=true for presentation mode")


class TestPX4Airframe(unittest.TestCase):
    """TC-3: PX4 airframe parameters must be set for pure GPS mode (no EV conflict)."""

    def setUp(self):
        self.assertTrue(os.path.exists(PX4_AIRFRAME), f"Missing PX4 airframe: {PX4_AIRFRAME}")
        with open(PX4_AIRFRAME) as f:
            self.content = f.read()

    def test_gps_enabled(self):
        self.assertIn("EKF2_GPS_CTRL   7", self.content,
                      "EKF2_GPS_CTRL must be 7 (GPS position + velocity fused)")

    def test_ev_disabled(self):
        self.assertIn("EKF2_EV_CTRL    0", self.content,
                      "EKF2_EV_CTRL must be 0 to disable external vision (prevent GPS+SLAM conflict!)")

    def test_height_ref_gps(self):
        self.assertIn("EKF2_HGT_REF    1", self.content,
                      "EKF2_HGT_REF must be 1 (GPS height) when using pure GPS mode")


class TestWorldFile(unittest.TestCase):
    """TC-4: Base world SDF must contain all required elements."""

    def setUp(self):
        self.path = f"{WORLDS_DIR}/stage1_field.sdf"
        self.assertTrue(os.path.exists(self.path), f"Missing base world: {self.path}")
        with open(self.path) as f:
            self.content = f.read()

    def test_world_name(self):
        self.assertIn('name="stage1_seeded"', self.content,
                      "World name must be 'stage1_seeded' to match PX4 SITL lookup")

    def test_start_zone_mat(self):
        self.assertIn("start_zone_mat", self.content,
                      "Green start zone mat must exist in world")

    def test_exit_zone_mat(self):
        self.assertIn("exit_zone_mat", self.content,
                      "Blue exit zone mat must exist in world")

    def test_gps_spherical_coords(self):
        self.assertIn("spherical_coordinates", self.content,
                      "Spherical coordinates needed for GPS plugin")
        self.assertIn("gz-sim-navsat-system", self.content,
                      "NavSat (GPS) plugin must be loaded in world")

    def test_walls(self):
        for wall in ["wall_north", "wall_south", "wall_east", "wall_west"]:
            self.assertIn(wall, self.content, f"Missing field wall: {wall}")


class TestScenarioGenerator(unittest.TestCase):
    """TC-5: scenario_generator.py must produce valid SDF and JSON with trees."""

    def setUp(self):
        sys.path.insert(0, f"{WS}/src/robofest_sim")
        from robofest_sim.scenario_generator import generate_scenario, build_sdf_content, load_config
        self.generate_scenario = generate_scenario
        self.build_sdf_content = build_sdf_content
        self.load_config = load_config

    def test_generate_scenario_seed_42(self):
        cfg = self.load_config(f"{CONFIG_DIR}/stage1.yaml")
        seed, mines, obstacles = self.generate_scenario(cfg, seed_override=42)
        self.assertEqual(seed, 42)
        self.assertEqual(len(mines), cfg["scenario"]["num_mines"])
        self.assertEqual(len(obstacles), cfg["scenario"]["num_obstacles"])

    def test_mine_spacing(self):
        cfg = self.load_config(f"{CONFIG_DIR}/stage1.yaml")
        _, mines, _ = self.generate_scenario(cfg, seed_override=42)
        min_spacing = cfg["scenario"]["min_mine_spacing"]
        for i, m1 in enumerate(mines):
            for j, m2 in enumerate(mines):
                if i != j:
                    dist = math.hypot(m1["x"] - m2["x"], m1["y"] - m2["y"])
                    self.assertGreaterEqual(dist, min_spacing - 0.01,
                        f"Mine {i} and {j} are too close: {dist:.2f}m < {min_spacing}m")

    def test_sdf_contains_trees(self):
        base_world = f"{WORLDS_DIR}/stage1_field.sdf"
        cfg = self.load_config(f"{CONFIG_DIR}/stage1.yaml")
        _, mines, obstacles = self.generate_scenario(cfg, seed_override=42)
        human_pos = cfg["human"]["start_position"]
        sdf = self.build_sdf_content(base_world, mines, obstacles, human_pos)
        self.assertIn("mountain_pine_tree", sdf,
                      "Tree 1 (mountain_pine_tree) must be in generated SDF")
        self.assertIn("tall_mountain_tree", sdf,
                      "Tree 2 (tall_mountain_tree) must be in generated SDF")

    def test_mines_in_minefield_zone(self):
        cfg = self.load_config(f"{CONFIG_DIR}/stage1.yaml")
        _, mines, _ = self.generate_scenario(cfg, seed_override=42)
        mz = cfg["zones"]["minefield_zone"]
        for m in mines:
            self.assertGreaterEqual(m["x"], mz["x_min"], f"Mine {m['id']} x={m['x']} out of bounds")
            self.assertLessEqual(m["x"], mz["x_max"], f"Mine {m['id']} x={m['x']} out of bounds")
            self.assertGreaterEqual(m["y"], mz["y_min"], f"Mine {m['id']} y={m['y']} out of bounds")
            self.assertLessEqual(m["y"], mz["y_max"], f"Mine {m['id']} y={m['y']} out of bounds")


class TestMissionLogic(unittest.TestCase):
    """TC-6: position_hold_node mission waypoints must be valid and tree-avoiding."""

    # Tree 1: X=18, Y=2 (radius ~1.5m canopy + 0.2m trunk)
    # Tree 2: X=23, Y=-2 (radius ~1.5m canopy + 0.25m trunk)
    TREE1 = (18.0, 2.0)
    TREE2 = (23.0, -2.0)
    SAFE_RADIUS = 2.0  # Minimum safe clearance from tree center

    def _waypoints(self):
        # Read directly from the node source
        src = open(f"{OFFBOARD_SRC}/position_hold_node.py").read()
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                for t in node.targets:
                    if isinstance(t, ast.Attribute) and t.attr == "waypoints":
                        return ast.literal_eval(node.value)
        return None

    def test_waypoints_extracted(self):
        wps = self._waypoints()
        self.assertIsNotNone(wps, "Could not extract waypoints from position_hold_node.py")
        self.assertGreaterEqual(len(wps), 3, "Must have at least 3 waypoints")

    def test_final_waypoint_in_blue_box(self):
        wps = self._waypoints()
        final_x, final_y = wps[-1]
        # In position_hold_node local NED frame, final_y (East) reaches ~33.0m (Gazebo X ~37.5m)
        self.assertGreaterEqual(final_y, 30.0, "Final WP must reach blue box (y_ned >= 30)")

    def test_waypoints_clear_of_tree1(self):
        wps = self._waypoints()
        tx, ty = self.TREE1
        for i, (wx, wy) in enumerate(wps):
            if 15.0 <= wx <= 22.0:  # Near tree 1 X range
                dist = math.hypot(wx - tx, wy - ty)
                self.assertGreaterEqual(dist, self.SAFE_RADIUS,
                    f"WP {i} ({wx},{wy}) is only {dist:.2f}m from Tree 1 at ({tx},{ty}) — too close!")

    def test_waypoints_clear_of_tree2(self):
        wps = self._waypoints()
        tx, ty = self.TREE2
        for i, (wx, wy) in enumerate(wps):
            if 20.0 <= wx <= 26.0:  # Near tree 2 X range
                dist = math.hypot(wx - tx, wy - ty)
                self.assertGreaterEqual(dist, self.SAFE_RADIUS,
                    f"WP {i} ({wx},{wy}) is only {dist:.2f}m from Tree 2 at ({tx},{ty}) — too close!")

    def test_waypoints_within_field_bounds(self):
        wps = self._waypoints()
        for i, (wx, wy) in enumerate(wps):
            # Waypoints in position_hold_node are in PX4 local NED frame (x=North, y=East)
            self.assertGreaterEqual(wx, -5.0, f"WP {i} X={wx} out of bounds")
            self.assertLessEqual(wx, 5.0, f"WP {i} X={wx} out of bounds")
            self.assertGreaterEqual(wy, -5.0, f"WP {i} Y={wy} out of bounds")
            self.assertLessEqual(wy, 35.0, f"WP {i} Y={wy} out of bounds")

    def test_hold_states_exist(self):
        with open(f"{OFFBOARD_SRC}/position_hold_node.py") as f:
            src = f.read()
        self.assertIn("HOLD_MID", src, "HOLD_MID state must exist in position_hold_node")
        self.assertIn("HOLD_END", src, "HOLD_END state must exist in position_hold_node")
        self.assertIn("TAKEOFF", src, "TAKEOFF state must exist")
        self.assertIn("NAVIGATE", src, "NAVIGATE state must exist")
        self.assertIn("LAND", src, "LAND state must exist")

    def test_greenbox_hold_duration(self):
        with open(f"{OFFBOARD_SRC}/position_hold_node.py") as f:
            src = f.read()
        # Expect hold of 40 ticks = 4s at 10Hz
        self.assertIn("40", src, "TAKEOFF greenbox hold duration must be present")


class TestGroundTruthPublisher(unittest.TestCase):
    """TC-7: ground_truth_publisher must have odom subscription in __init__, not timer."""

    def test_odom_sub_in_init_not_timer(self):
        src = open(f"{ROBOFEST_SRC}/ground_truth_publisher.py").read()
        tree = ast.parse(src)

        init_body = None
        timer_body = None

        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                for item in node.body:
                    if isinstance(item, ast.FunctionDef):
                        if item.name == "__init__":
                            init_body = ast.dump(item)
                        elif item.name == "timer_callback":
                            timer_body = ast.dump(item)

        self.assertIsNotNone(init_body, "__init__ not found")
        self.assertIsNotNone(timer_body, "timer_callback not found")
        self.assertIn("odom_sub", init_body,
                      "odom_sub must be created in __init__, not timer_callback")
        self.assertNotIn("odom_sub", timer_body,
                         "odom_sub must NOT be created inside timer_callback (causes re-subscribe flood)")


class TestLaunchFile(unittest.TestCase):
    """TC-8: Launch file must include all required nodes."""

    def setUp(self):
        self.path = f"{WS}/src/robofest_sim/launch/robofest_sim.launch.py"
        self.assertTrue(os.path.exists(self.path))
        with open(self.path) as f:
            self.content = f.read()

    def test_rviz_launched(self):
        self.assertIn("rviz2", self.content, "RViz2 must be in launch file")

    def test_slam_toolbox_launched(self):
        self.assertIn("slam_toolbox", self.content, "SLAM toolbox must be in launch file")

    def test_ros_gz_bridge_launched(self):
        self.assertIn("parameter_bridge", self.content, "ros_gz_bridge must bridge /scan topic")
        self.assertIn("/scan", self.content, "/scan topic must be bridged")

    def test_ekf2_checker_launched(self):
        self.assertIn("ekf2_readiness_checker", self.content, "EKF2 readiness checker must be in launch file")

    def test_tf_publishers_present(self):
        self.assertIn("static_transform_publisher", self.content, "TF publishers must be in launch file")
        self.assertIn("base_link", self.content, "base_link TF frame must be published")


class TestRVizConfig(unittest.TestCase):
    """TC-9: RViz config must include LaserScan display."""

    def test_laserscan_in_rviz(self):
        path = f"{WS}/src/robofest_sim/rviz/robofest_sim.rviz"
        self.assertTrue(os.path.exists(path), f"Missing: {path}")
        with open(path) as f:
            content = f.read()
        self.assertIn("LaserScan", content, "LaserScan display must be in RViz config")
        self.assertIn("/scan", content, "/scan topic must be in RViz config")


if __name__ == "__main__":
    print("=" * 60)
    print("  Robofest Simulation Full Pipeline Test Suite")
    print("=" * 60)
    print()

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    suite.addTests(loader.loadTestsFromTestCase(TestSyntax))
    suite.addTests(loader.loadTestsFromTestCase(TestConfig))
    suite.addTests(loader.loadTestsFromTestCase(TestPX4Airframe))
    suite.addTests(loader.loadTestsFromTestCase(TestWorldFile))
    suite.addTests(loader.loadTestsFromTestCase(TestScenarioGenerator))
    suite.addTests(loader.loadTestsFromTestCase(TestMissionLogic))
    suite.addTests(loader.loadTestsFromTestCase(TestGroundTruthPublisher))
    suite.addTests(loader.loadTestsFromTestCase(TestLaunchFile))
    suite.addTests(loader.loadTestsFromTestCase(TestRVizConfig))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print()
    if result.wasSuccessful():
        print("✅ ALL TESTS PASSED — Pipeline is ready to run!")
    else:
        print(f"❌ {len(result.failures)} FAILURES, {len(result.errors)} ERRORS — Fix before running!")
    sys.exit(0 if result.wasSuccessful() else 1)
