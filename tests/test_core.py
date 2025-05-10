"""
Tests for the core deconfliction functionality.
"""
import sys
import os
import unittest
from datetime import datetime, timedelta
import numpy as np

# Add the parent directory to system path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from deconfliction import Waypoint, Mission, DeconflictionSystem, ConflictResult


class TestDeconflictionCore(unittest.TestCase):
    """Test cases for the core deconfliction functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.start_time = datetime.now()
        
        # Create some test missions
        self.mission1 = Mission(
            waypoints=[
                Waypoint(x=0, y=0, time=self.start_time),
                Waypoint(x=100, y=0, time=self.start_time + timedelta(minutes=5)),
            ],
            start_time=self.start_time,
            end_time=self.start_time + timedelta(minutes=5),
            drone_id="drone1"
        )
        
        self.mission2 = Mission(
            waypoints=[
                Waypoint(x=0, y=100, time=self.start_time),
                Waypoint(x=100, y=100, time=self.start_time + timedelta(minutes=5)),
            ],
            start_time=self.start_time,
            end_time=self.start_time + timedelta(minutes=5),
            drone_id="drone2"
        )
        
        self.mission3 = Mission(
            waypoints=[
                Waypoint(x=0, y=0, time=self.start_time + timedelta(minutes=10)),
                Waypoint(x=100, y=0, time=self.start_time + timedelta(minutes=15)),
            ],
            start_time=self.start_time + timedelta(minutes=10),
            end_time=self.start_time + timedelta(minutes=15),
            drone_id="drone3"
        )
        
        # Conflicting mission (crosses mission1)
        self.conflict_mission = Mission(
            waypoints=[
                Waypoint(x=50, y=-50, time=self.start_time + timedelta(minutes=2)),
                Waypoint(x=50, y=50, time=self.start_time + timedelta(minutes=3)),
            ],
            start_time=self.start_time + timedelta(minutes=2),
            end_time=self.start_time + timedelta(minutes=3),
            drone_id="conflict"
        )
        
        # System with minimum safe distance of 10 meters
        self.system = DeconflictionSystem(
            [self.mission1, self.mission2, self.mission3],
            min_safe_distance=10.0
        )
    
    def test_no_conflict_different_space(self):
        """Test that missions in different spaces don't conflict."""
        # Mission in a completely different area
        test_mission = Mission(
            waypoints=[
                Waypoint(x=1000, y=1000, time=self.start_time),
                Waypoint(x=1100, y=1000, time=self.start_time + timedelta(minutes=5)),
            ],
            start_time=self.start_time,
            end_time=self.start_time + timedelta(minutes=5),
            drone_id="test"
        )
        
        result = self.system.check_mission(test_mission)
        self.assertTrue(result.is_clear)
        self.assertEqual(len(result.conflict_details), 0)
    
    def test_no_conflict_different_time(self):
        """Test that missions at different times don't conflict."""
        # Mission in the same space but at a different time
        test_mission = Mission(
            waypoints=[
                Waypoint(x=0, y=0, time=self.start_time + timedelta(minutes=20)),
                Waypoint(x=100, y=0, time=self.start_time + timedelta(minutes=25)),
            ],
            start_time=self.start_time + timedelta(minutes=20),
            end_time=self.start_time + timedelta(minutes=25),
            drone_id="test"
        )
        
        result = self.system.check_mission(test_mission)
        self.assertTrue(result.is_clear)
        self.assertEqual(len(result.conflict_details), 0)
    
    def test_conflict_detection(self):
        """Test that conflicts are correctly detected."""
        result = self.system.check_mission(self.conflict_mission)
        
        self.assertFalse(result.is_clear)
        self.assertGreater(len(result.conflict_details), 0)
        
        # Verify conflict details
        conflict = result.conflict_details[0]
        self.assertEqual(conflict['conflicting_drone_id'], 'drone1')
        self.assertLess(conflict['actual_distance'], self.system.min_safe_distance)
    
    def test_min_distance_calculation(self):
        """Test the minimum distance calculation between line segments."""
        # Two perpendicular lines, closest at (5,0) and (5,5) with distance 5
        p1 = np.array([0, 0])
        p2 = np.array([10, 0])
        p3 = np.array([5, 5])
        p4 = np.array([5, 10])
        
        dist, pt1, pt2 = self.system._min_distance_segments(p1, p2, p3, p4)
        
        self.assertAlmostEqual(dist, 5.0)
        np.testing.assert_almost_equal(pt1, np.array([5, 0]))
        np.testing.assert_almost_equal(pt2, np.array([5, 5]))
    
    def test_same_drone_no_conflict(self):
        """Test that a drone doesn't conflict with itself."""
        # Create a copy of mission1 with the same drone_id
        same_drone = Mission(
            waypoints=[
                Waypoint(x=10, y=10, time=self.start_time),
                Waypoint(x=90, y=90, time=self.start_time + timedelta(minutes=5)),
            ],
            start_time=self.start_time,
            end_time=self.start_time + timedelta(minutes=5),
            drone_id="drone1"  # Same as mission1
        )
        
        result = self.system.check_mission(same_drone)
        self.assertTrue(result.is_clear)
    
    def test_3d_missions(self):
        """Test conflict detection with 3D coordinates (altitude)."""
        # Create 3D missions with sufficient altitude separation
        mission_3d_low = Mission(
            waypoints=[
                Waypoint(x=0, y=0, z=0, time=self.start_time),
                Waypoint(x=100, y=0, z=0, time=self.start_time + timedelta(minutes=5)),
            ],
            start_time=self.start_time,
            end_time=self.start_time + timedelta(minutes=5),
            drone_id="low"
        )
        
        mission_3d_high = Mission(
            waypoints=[
                Waypoint(x=50, y=0, z=100, time=self.start_time + timedelta(minutes=2)),
                Waypoint(x=50, y=100, z=100, time=self.start_time + timedelta(minutes=3)),
            ],
            start_time=self.start_time + timedelta(minutes=2),
            end_time=self.start_time + timedelta(minutes=3),
            drone_id="high"
        )
        
        # System with 3D missions
        system_3d = DeconflictionSystem([mission_3d_low], min_safe_distance=20.0)
        
        # Check mission - should be clear due to altitude separation
        result = system_3d.check_mission(mission_3d_high)
        self.assertTrue(result.is_clear)
        
        # Now make a mission with insufficient altitude separation
        mission_3d_conflict = Mission(
            waypoints=[
                Waypoint(x=50, y=0, z=15, time=self.start_time + timedelta(minutes=2)),
                Waypoint(x=50, y=100, z=15, time=self.start_time + timedelta(minutes=3)),
            ],
            start_time=self.start_time + timedelta(minutes=2),
            end_time=self.start_time + timedelta(minutes=3),
            drone_id="conflict"
        )
        
        # Should detect a conflict
        result = system_3d.check_mission(mission_3d_conflict)
        self.assertFalse(result.is_clear)


if __name__ == '__main__':
    unittest.main() 