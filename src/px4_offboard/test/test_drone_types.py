import pytest
from px4_offboard.drone_types import HumanEscapePath, Waypoint, WaypointVerdict

def test_human_escape_path():
    path = HumanEscapePath(waypoints=[
        Waypoint(x=0.0, y=0.0),
        Waypoint(x=10.0, y=0.5),
        Waypoint(x=20.0, y=-0.5)
    ])
    
    assert len(path.waypoints) == 3
    assert path.overall_verdict is None
    
    # Initialize verdicts list
    path.reset_verdicts()
    
    # Record some verdicts
    path.record_verdict(0, True)
    path.record_verdict(1, False)
    
    # Check intermediate state
    assert path.verdicts[0] == WaypointVerdict.PASS
    assert path.verdicts[1] == WaypointVerdict.FAIL
    assert path.verdicts[2] == WaypointVerdict.PENDING
    
    # Finalise should return False because one is UNSAFE and one is PENDING
    assert not path.finalise()
    assert path.overall_verdict is False
    
    # Reset and try again
    path.reset_verdicts()
    assert path.verdicts[0] == WaypointVerdict.PENDING
    
    path.record_verdict(0, True)
    path.record_verdict(1, True)
    path.record_verdict(2, True)
    
    assert path.finalise()
    assert path.overall_verdict is True
