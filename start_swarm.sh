#!/bin/bash
# ============================================================
# start_swarm.sh — Launch 3 PX4 SITL drones + 3 XRCE agents
# ============================================================
# Usage: bash ~/px4_ros2_ws/start_swarm.sh
#
# This script:
#  1. Kills any lingering PX4/Gazebo/XRCE processes
#  2. Starts 3 MicroXRCE agents (ports 8888, 8889, 8890)
#  3. Waits for the user to launch Drone 0 via make (in its own terminal)
#  4. Spawns Drones 1 and 2 via direct PX4 binary
# ============================================================

PX4_DIR="$HOME/PX4-Autopilot"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
WORLD="stage1_seeded"

# ── Step 0: Kill everything cleanly ──────────────────────────
echo ""
echo "════════════════════════════════════════════════════"
echo "  Cleaning up old PX4 / Gazebo / XRCE processes..."
echo "════════════════════════════════════════════════════"
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -x px4             2>/dev/null || true
pkill -9 -f ruby            2>/dev/null || true
pkill -9 -f gzserver        2>/dev/null || true
pkill -9 -f gz_sim          2>/dev/null || true
sleep 2

# ── Step 1: Build check ──────────────────────────────────────
if [ ! -f "$BUILD_DIR/bin/px4" ]; then
    echo "ERROR: PX4 binary not found. Run 'make px4_sitl gz_x500_lidar_2d' first."
    exit 1
fi

# ── Step 2: Start ONE MicroXRCE agent (all 3 drones use port 8888) ──────────
# PX4 SITL multi-instance: ALL instances connect to port 8888 with different
# DDS keys (key=1, key=2, key=3). Namespaces are auto-set by PX4 as px4_1, px4_2.
echo ""
echo "Starting MicroXRCE Agent on port 8888 (shared by all 3 drones)..."
MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_0.log 2>&1 &
sleep 2
echo "  ✅ XRCE Agent → port 8888 (all drones share this)"

# ── Step 3: Launch Drone 0 (opens Gazebo) ───────────────────
echo ""
echo "════════════════════════════════════════════════════"
echo "  Spawning Drone 0 — South lane (Y=-2)..."
echo "════════════════════════════════════════════════════"
cd "$PX4_DIR"
PX4_GZ_WORLD="$WORLD" \
PX4_GZ_MODEL_POSE="4.5,-2,0.25" \
PX4_GZ_INSTANCE=0 \
make px4_sitl gz_x500_lidar_2d &
DRONE0_PID=$!

echo ""
echo "  Waiting 35s for Gazebo to fully start..."
sleep 30

# ── Step 4: Spawn Drone 1 (center lane) ─────────────────────
echo ""
echo "════════════════════════════════════════════════════"
echo "  Spawning Drone 1 — Centre lane (Y=0)..."
echo "════════════════════════════════════════════════════"
mkdir -p "$BUILD_DIR/instance_1"
cd "$BUILD_DIR/instance_1"
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD="$WORLD" \
PX4_GZ_MODEL_POSE="4.5,0,0.25" \
PX4_GZ_INSTANCE=1 \
PX4_SIM_MODEL=gz_x500_lidar_2d \
"$BUILD_DIR/bin/px4" -i 1 -d "$BUILD_DIR/etc" > /tmp/px4_1.log 2>&1 &
DRONE1_PID=$!
echo "  Drone 1 PID: $DRONE1_PID"
sleep 20

# ── Step 5: Spawn Drone 2 (north lane) ──────────────────────
echo ""
echo "════════════════════════════════════════════════════"
echo "  Spawning Drone 2 — North lane (Y=+2)..."
echo "════════════════════════════════════════════════════"
mkdir -p "$BUILD_DIR/instance_2"
cd "$BUILD_DIR/instance_2"
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD="$WORLD" \
PX4_GZ_MODEL_POSE="4.5,2,0.25" \
PX4_GZ_INSTANCE=2 \
PX4_SIM_MODEL=gz_x500_lidar_2d \
"$BUILD_DIR/bin/px4" -i 2 -d "$BUILD_DIR/etc" > /tmp/px4_2.log 2>&1 &
DRONE2_PID=$!
echo "  Drone 2 PID: $DRONE2_PID"
sleep 20
echo ""
echo "  Waiting 10s more for all drones to fully initialize EKF2..."
sleep 10

echo ""
echo "════════════════════════════════════════════════════"
echo "  ✅ All 3 drones spawned!"
echo "     Drone 0 (South,  Y=-2): instance 0, port 8888, /fmu/..."
echo "     Drone 1 (Centre, Y= 0): instance 1, port 8889, /px4_1/fmu/..."
echo "     Drone 2 (North,  Y=+2): instance 2, port 8890, /px4_2/fmu/..."
echo ""
echo "  Now in a NEW terminal, run:"
echo "    cd ~/px4_ros2_ws && source install/setup.bash"
echo "    ros2 launch robofest_sim robofest_sim.launch.py seed:=42 rviz:=true"
echo ""
echo "  Then in ANOTHER terminal, start the swarm:"
echo "    cd ~/px4_ros2_ws && source install/setup.bash"
echo "    ros2 run px4_offboard swarm_mission_node"
echo ""
echo "  To verify all 3 drones have DDS topics:"
echo "    ros2 topic list | grep fmu | sort"
echo "  Expected: /fmu/..., /px4_1/fmu/..., /px4_2/fmu/..."
echo "════════════════════════════════════════════════════"

# Keep script alive
wait $DRONE0_PID
