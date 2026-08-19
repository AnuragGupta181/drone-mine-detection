#!/usr/bin/env python3

import os
import sys
import yaml
import json
import random
import math
import argparse
from datetime import datetime
from collections import deque

def parse_args():
    parser = argparse.ArgumentParser(description="Stage 1 Scenario Generator for Competition Simulation")
    parser.add_argument("--config", type=str, default="", help="Path to stage1.yaml config file")
    parser.add_argument("--seed", type=int, default=None, help="Random seed override")
    parser.add_argument("--mines", type=int, default=None, help="Mine count override")
    parser.add_argument("--obstacles", type=int, default=None, help="Obstacle count override")
    parser.add_argument("--output-sdf", type=str, default="", help="Output SDF file path")
    parser.add_argument("--output-json", type=str, default="", help="Output ground truth JSON path")
    return parser.parse_args()

def load_config(config_path):
    if not config_path:
        # Default package location search
        possible_paths = [
            os.path.join(os.path.dirname(__file__), "..", "config", "stage1.yaml"),
            "/home/ubuntu/px4_ros2_ws/src/competition_sim/config/stage1.yaml"
        ]
        for p in possible_paths:
            if os.path.exists(p):
                config_path = p
                break
    
    if not os.path.exists(config_path):
        print(f"[scenario_generator] ERROR: Config file not found at {config_path}")
        sys.exit(1)
        
    with open(config_path, "r") as f:
        return yaml.safe_load(f)

def check_navigable_corridor(field_length, field_width, mines, obstacles, min_clearance=0.8):
    """
    Grid-based BFS connectivity check to ensure at least one navigable path exists 
    from Start Zone (X=2.5, Y=0.0) to Exit Zone (X=37.5, Y=0.0).
    """
    grid_res = 0.5  # 0.5m grid resolution
    x_steps = int(field_length / grid_res)
    y_min, y_max = -field_width / 2.0, field_width / 2.0
    y_steps = int(field_width / grid_res)
    
    start_gx = int(2.5 / grid_res)
    start_gy = int((0.0 - y_min) / grid_res)
    exit_gx = int(37.5 / grid_res)
    
    # Create grid: True = walkable, False = blocked by obstacle/mine clearance
    grid = [[True for _ in range(y_steps)] for _ in range(x_steps)]
    
    for gx in range(x_steps):
        cx = gx * grid_res
        for gy in range(y_steps):
            cy = y_min + gy * grid_res
            
            # Boundary buffer check
            if abs(cy) > (field_width / 2.0 - 0.3):
                grid[gx][gy] = False
                continue
                
            # Check mines
            for m in mines:
                dist = math.hypot(cx - m["x"], cy - m["y"])
                if dist < min_clearance:
                    grid[gx][gy] = False
                    break
                    
            if not grid[gx][gy]:
                continue
                
            # Check obstacles
            for obs in obstacles:
                dist = math.hypot(cx - obs["x"], cy - obs["y"])
                if dist < (min_clearance + 0.4): # obstacle radius ~0.4
                    grid[gx][gy] = False
                    break

    # BFS search from start to any cell in exit zone (gx >= exit_gx)
    queue = deque([(start_gx, start_gy)])
    visited = set([(start_gx, start_gy)])
    
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    
    while queue:
        cx, cy = queue.popleft()
        if cx >= exit_gx:
            return True # Path found!
            
        for dx, dy in directions:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < x_steps and 0 <= ny < y_steps:
                if grid[nx][ny] and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
                    
    return False # No valid path found

def generate_scenario(cfg, seed_override=None, mine_override=None, obstacle_override=None):
    seed = seed_override if seed_override is not None else cfg["scenario"]["seed"]
    num_mines = mine_override if mine_override is not None else cfg["scenario"]["num_mines"]
    num_obstacles = obstacle_override if obstacle_override is not None else cfg["scenario"]["num_obstacles"]
    
    field_length = cfg["field"]["length"]
    field_width = cfg["field"]["width"]
    
    mf_zone = cfg["zones"]["minefield_zone"]
    min_m_spacing = cfg["scenario"]["min_mine_spacing"]
    min_obs_spacing = cfg["scenario"]["min_obstacle_spacing"]
    
    random.seed(seed)
    
    max_attempts = 200
    valid_layout = False
    
    mines = []
    obstacles = []
    
    for attempt in range(max_attempts):
        mines = []
        obstacles = []
        
        # 1. Generate Static Obstacles
        obs_attempts = 0
        while len(obstacles) < num_obstacles and obs_attempts < 500:
            obs_attempts += 1
            ox = random.uniform(mf_zone["x_min"] + 2.0, mf_zone["x_max"] - 2.0)
            oy = random.uniform(mf_zone["y_min"] + 1.0, mf_zone["y_max"] - 1.0)
            
            overlap = False
            for prev_o in obstacles:
                if math.hypot(ox - prev_o["x"], oy - prev_o["y"]) < min_obs_spacing:
                    overlap = True
                    break
            if not overlap:
                obstacles.append({"id": f"obstacle_{len(obstacles)+1}", "x": round(ox, 2), "y": round(oy, 2), "z": 0.0})
                
        # 2. Generate Simulated Mines (Traffic Cones)
        mine_attempts = 0
        while len(mines) < num_mines and mine_attempts < 1000:
            mine_attempts += 1
            mx = random.uniform(mf_zone["x_min"], mf_zone["x_max"])
            my = random.uniform(mf_zone["y_min"], mf_zone["y_max"])
            
            overlap = False
            # Spacing to existing mines
            for prev_m in mines:
                if math.hypot(mx - prev_m["x"], my - prev_m["y"]) < min_m_spacing:
                    overlap = True
                    break
            if overlap:
                continue
                
            # Spacing to obstacles
            for prev_o in obstacles:
                if math.hypot(mx - prev_o["x"], my - prev_o["y"]) < (min_m_spacing + 0.5):
                    overlap = True
                    break
            if not overlap:
                mines.append({
                    "id": f"mine_{len(mines)+1}",
                    "model_type": cfg["scenario"]["mine_model_type"],
                    "x": round(mx, 2),
                    "y": round(my, 2),
                    "z": 0.0
                })

        if len(mines) == num_mines and len(obstacles) == num_obstacles:
            # Check corridor connectivity
            if check_navigable_corridor(field_length, field_width, mines, obstacles):
                valid_layout = True
                print(f"[scenario_generator] Valid layout generated on attempt {attempt+1} (Seed={seed}).")
                break

    if not valid_layout:
        print(f"[scenario_generator] WARNING: Could not satisfy strict placement in {max_attempts} attempts. Returning best effort.")
        
    return seed, mines, obstacles

def build_sdf_content(base_world_path, mines, obstacles, human_pos, appearance_mode="easy"):
    with open(base_world_path, "r") as f:
        world_sdf = f.read()

    sdf_insertions = []
    
    # 1. Human Model Inclusion
    sdf_insertions.append(f"""
    <!-- Static Human Model -->
    <include>
      <uri>model://static_human</uri>
      <name>human_0</name>
      <pose>{human_pos['x']} {human_pos['y']} {human_pos['z']} 0 0 {human_pos['yaw']}</pose>
    </include>
""")

    # 2. Obstacle Inclusion
    for obs in obstacles:
        sdf_insertions.append(f"""
    <include>
      <uri>model://static_obstacle</uri>
      <name>{obs['id']}</name>
      <pose>{obs['x']} {obs['y']} {obs['z']} 0 0 0</pose>
    </include>
""")

    # 3. Simulated Mines (Traffic Cones) Inclusion
    for m in mines:
        sdf_insertions.append(f"""
    <include>
      <uri>model://traffic_cone</uri>
      <name>{m['id']}</name>
      <pose>{m['x']} {m['y']} {m['z']} 0 0 0</pose>
    </include>
""")

    insertion_str = "\n".join(sdf_insertions)
    
    # Insert before </world> tag
    world_end_idx = world_sdf.rfind("</world>")
    if world_end_idx != -1:
        final_sdf = world_sdf[:world_end_idx] + insertion_str + "\n  " + world_sdf[world_end_idx:]
    else:
        final_sdf = world_sdf + insertion_str

    return final_sdf

def main():
    args = parse_args()
    cfg = load_config(args.config)
    
    seed, mines, obstacles = generate_scenario(
        cfg, 
        seed_override=args.seed,
        mine_override=args.mines,
        obstacle_override=args.obstacles
    )
    
    human_pos = cfg["human"]["start_position"]
    
    # Build output file paths
    pkg_dir = "/home/ubuntu/px4_ros2_ws/src/competition_sim"
    base_world_path = os.path.join(pkg_dir, "worlds", "stage1_field.sdf")
    
    out_sdf_path = args.output_sdf if args.output_sdf else os.path.join(pkg_dir, "worlds", "generated", "stage1_seeded.sdf")
    out_json_path = args.output_json if args.output_json else os.path.join(pkg_dir, "worlds", "generated", "stage1_manifest.json")
    
    os.makedirs(os.path.dirname(out_sdf_path), exist_ok=True)
    os.makedirs(os.path.dirname(out_json_path), exist_ok=True)
    
    # Generate SDF
    appearance_mode = cfg["scenario"].get("mine_appearance_mode", "easy")
    final_sdf = build_sdf_content(base_world_path, mines, obstacles, human_pos, appearance_mode)
    with open(out_sdf_path, "w") as f:
        f.write(final_sdf)
    print(f"[scenario_generator] Exported SDF world to: {out_sdf_path}")
    
    # Generate Ground Truth Manifest
    manifest = {
        "scenario_id": f"stage1_seed_{seed}_{int(datetime.now().timestamp())}",
        "seed": seed,
        "timestamp": datetime.now().isoformat(),
        "field": cfg["field"],
        "zones": cfg["zones"],
        "human": {
            "id": "human_0",
            "position": [human_pos["x"], human_pos["y"], human_pos["z"]],
            "yaw": human_pos["yaw"]
        },
        "mines": [
            {
                "id": m["id"],
                "model_type": m["model_type"],
                "position": [m["x"], m["y"], m["z"]],
                "radius": 0.15,
                "clearance_required": 1.0
            } for m in mines
        ],
        "obstacles": [
            {
                "id": o["id"],
                "position": [o["x"], o["y"], o["z"]],
                "size": [0.8, 0.8, 1.5]
            } for o in obstacles
        ]
    }
    
    with open(out_json_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"[scenario_generator] Exported ground truth manifest to: {out_json_path}")

if __name__ == "__main__":
    main()
