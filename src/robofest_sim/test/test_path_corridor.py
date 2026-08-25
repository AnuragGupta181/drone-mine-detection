import pytest
from robofest_sim.path_corridor import (
    ClearanceValidator, AverageYStrategy, MaxClearanceStrategy, CorridorMerger
)

# ── Test Data ────────────────────────────────────────────────────────────────
MINES = [
    {"id": "mine_1", "position": [10.0, 1.0, 0.0]},
    {"id": "mine_2", "position": [20.0, -2.0, 0.0]},
]

PATH_SOUTH = [(0, -2), (10, -2.5), (20, -2), (30, -2)]
PATH_CENTRE = [(0, 0), (10, 0), (20, 0), (30, 0)]
PATH_NORTH = [(0, 2), (10, 2), (20, 2.5), (30, 2)]

# ── Tests ────────────────────────────────────────────────────────────────────
def test_clearance_at():
    # Exactly on mine 1
    assert ClearanceValidator.clearance_at(10.0, 1.0, MINES) == 0.0
    
    # 3 meters away from mine 1 (10, 1) -> (10, 4)
    assert ClearanceValidator.clearance_at(10.0, 4.0, MINES) == 3.0
    
    # Empty mines
    assert ClearanceValidator.clearance_at(0.0, 0.0, []) == float('inf')

def test_clearance_validate():
    # A path that hits mine 1
    bad_path = [(0, 1), (10, 1), (20, 1)]
    ok, report = ClearanceValidator.validate(bad_path, MINES, min_clearance_m=1.0)
    assert not ok
    assert "FAIL" in report
    
    # A perfectly safe path down Y=-4
    good_path = [(0, -4), (10, -4), (20, -4)]
    ok, report = ClearanceValidator.validate(good_path, MINES, min_clearance_m=1.0)
    assert ok
    assert "clear mines" in report

def test_average_y_strategy():
    strategy = AverageYStrategy()
    # At X=10, Ys are: -2.5, 0, +2 => Avg = -0.5 / 3 = -0.1666...
    path_out = strategy.merge_paths([PATH_SOUTH, PATH_CENTRE, PATH_NORTH], 10.0, 10.0, 1.0)
    y_out = path_out[0][1]
    assert pytest.approx(y_out, 0.01) == -0.166

def test_max_clearance_strategy():
    strategy = MaxClearanceStrategy(MINES)
    
    # At X=10, the mines are at (10, 1) and (20, -2).
    # Path South: (10, -2.5). Dist to (10,1) = 3.5. Dist to (20,-2) = 10.01
    # Path Centre: (10, 0). Dist to (10,1) = 1.0. 
    # Path North: (10, 2). Dist to (10,1) = 1.0.
    # Therefore, South has max clearance (3.5m)
    path_out = strategy.merge_paths([PATH_SOUTH, PATH_CENTRE, PATH_NORTH], 10.0, 10.0, 1.0)
    y_out = path_out[0][1]
    assert y_out == -2.5

def test_corridor_merger_integration():
    strategy = MaxClearanceStrategy(MINES)
    merger = CorridorMerger(x_sample_step=10.0, strategy=strategy)
    
    merged_path = merger.merge([PATH_SOUTH, PATH_CENTRE, PATH_NORTH], x_start=0, x_end=30)
    
    assert len(merged_path) > 0
    # First point should be at X=0
    assert merged_path[0][0] == 0.0
    # The X coordinates should be strictly increasing
    for i in range(1, len(merged_path)):
        assert merged_path[i][0] > merged_path[i-1][0]
