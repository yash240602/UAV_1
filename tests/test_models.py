"""
Tests for the drone mission data models.
"""
import sys
import os
import unittest
from datetime import datetime, timedelta
import numpy as np

# Add the parent directory to system path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from deconfliction import Waypoint, Mission, ConflictResult


class TestWaypoint(unittest.TestCase):
    """Test cases for the Waypoint class."""
    
    def test_waypoint_creation(self):
        """Test basic waypoint creation."""
        wp = Waypoint(x=10.0, y=20.0)
        self.assertEqual(wp.x, 10.0)
        self.assertEqual(wp.y, 20.0)
        self.assertIsNone(wp.z)
        self.assertIsNone(wp.time)
        
        # With optional parameters
        now = datetime.now()
        wp = Waypoint(x=10.0, y=20.0, z=30.0, time=now)
        self.assertEqual(wp.x, 10.0)
        self.assertEqual(wp.y, 20.0)
        self.assertEqual(wp.z, 30.0)
        self.assertEqual(wp.time, now)
    
    def test_waypoint_coordinates(self):
        """Test the coordinates method."""
        wp = Waypoint(x=10.0, y=20.0)
        coords = wp.coordinates()
        self.assertIsInstance(coords, np.ndarray)
        np.testing.assert_array_equal(coords, np.array([10.0, 20.0]))
        
        # With z-coordinate
        wp = Waypoint(x=10.0, y=20.0, z=30.0)
        coords = wp.coordinates(include_z=True)
        np.testing.assert_array_equal(coords, np.array([10.0, 20.0, 30.0]))
        
        # Without z-coordinate even when z is available
        coords = wp.coordinates(include_z=False)
        np.testing.assert_array_equal(coords, np.array([10.0, 20.0]))
    
    def test_waypoint_distance(self):
        """Test distance calculation between waypoints."""
        wp1 = Waypoint(x=0.0, y=0.0)
        wp2 = Waypoint(x=3.0, y=4.0)
        
        # Distance should be 5.0 (3-4-5 triangle)
        self.assertEqual(wp1.distance_to(wp2), 5.0)
        
        # Check symmetry
        self.assertEqual(wp2.distance_to(wp1), 5.0)
        
        # With z-coordinate
        wp3 = Waypoint(x=0.0, y=0.0, z=0.0)
        wp4 = Waypoint(x=3.0, y=4.0, z=12.0)
        
        # Distance in 3D should be 13.0 (5^2 + 12^2 = 13^2)
        self.assertEqual(wp3.distance_to(wp4, include_z=True), 13.0)
        
        # Without z-coordinate should still be 5.0
        self.assertEqual(wp3.distance_to(wp4, include_z=False), 5.0)


class TestMission(unittest.TestCase):
    """Test cases for the Mission class."""
    
    def test_mission_creation(self):
        """Test basic mission creation."""
        start_time = datetime.now()
        end_time = start_time + timedelta(minutes=10)
        
        waypoints = [
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=100.0, y=0.0),
            Waypoint(x=100.0, y=100.0)
        ]
        
        mission = Mission(
            waypoints=waypoints,
            start_time=start_time,
            end_time=end_time,
            drone_id="test_drone"
        )
        
        self.assertEqual(len(mission.waypoints), 3)
        self.assertEqual(mission.start_time, start_time)
        self.assertEqual(mission.end_time, end_time)
        self.assertEqual(mission.drone_id, "test_drone")
    
    def test_mission_validation(self):
        """Test mission validation during creation."""
        start_time = datetime.now()
        end_time = start_time + timedelta(minutes=10)
        
        # Empty waypoints should raise ValueError
        with self.assertRaises(ValueError):
            Mission(
                waypoints=[],
                start_time=start_time,
                end_time=end_time,
                drone_id="test_drone"
            )
        
        # End time before start time should raise ValueError
        with self.assertRaises(ValueError):
            Mission(
                waypoints=[Waypoint(x=0.0, y=0.0)],
                start_time=end_time,  # Swapped
                end_time=start_time,  # Swapped
                drone_id="test_drone"
            )
    
    def test_waypoint_time_assignment(self):
        """Test automatic assignment of waypoint times."""
        start_time = datetime.now()
        end_time = start_time + timedelta(minutes=10)
        
        waypoints = [
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=50.0, y=0.0),
            Waypoint(x=100.0, y=0.0)
        ]
        
        mission = Mission(
            waypoints=waypoints,
            start_time=start_time,
            end_time=end_time,
            drone_id="test_drone"
        )
        
        # Check that times were assigned to waypoints
        self.assertEqual(mission.waypoints[0].time, start_time)
        self.assertEqual(
            mission.waypoints[1].time, 
            start_time + timedelta(minutes=5)
        )
        self.assertEqual(mission.waypoints[2].time, end_time)
    
    def test_get_path_segments(self):
        """Test getting path segments from a mission."""
        start_time = datetime.now()
        end_time = start_time + timedelta(minutes=10)
        
        waypoints = [
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=50.0, y=0.0),
            Waypoint(x=100.0, y=0.0)
        ]
        
        mission = Mission(
            waypoints=waypoints,
            start_time=start_time,
            end_time=end_time,
            drone_id="test_drone"
        )
        
        segments = mission.get_path_segments()
        
        # Should have 2 segments for 3 waypoints
        self.assertEqual(len(segments), 2)
        
        # Check segment pairs
        self.assertEqual(segments[0][0], mission.waypoints[0])
        self.assertEqual(segments[0][1], mission.waypoints[1])
        self.assertEqual(segments[1][0], mission.waypoints[1])
        self.assertEqual(segments[1][1], mission.waypoints[2])
    
    def test_is_3d(self):
        """Test 3D mission detection."""
        start_time = datetime.now()
        end_time = start_time + timedelta(minutes=10)
        
        # 2D mission (no z coordinates)
        waypoints_2d = [
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=100.0, y=0.0)
        ]
        
        mission_2d = Mission(
            waypoints=waypoints_2d,
            start_time=start_time,
            end_time=end_time,
            drone_id="test_2d"
        )
        
        self.assertFalse(mission_2d.is_3d())
        
        # 3D mission (all waypoints have z coordinates)
        waypoints_3d = [
            Waypoint(x=0.0, y=0.0, z=0.0),
            Waypoint(x=100.0, y=0.0, z=100.0)
        ]
        
        mission_3d = Mission(
            waypoints=waypoints_3d,
            start_time=start_time,
            end_time=end_time,
            drone_id="test_3d"
        )
        
        self.assertTrue(mission_3d.is_3d())
        
        # Mixed mission (some waypoints have z, some don't)
        waypoints_mixed = [
            Waypoint(x=0.0, y=0.0, z=0.0),
            Waypoint(x=100.0, y=0.0)  # No z
        ]
        
        mission_mixed = Mission(
            waypoints=waypoints_mixed,
            start_time=start_time,
            end_time=end_time,
            drone_id="test_mixed"
        )
        
        self.assertFalse(mission_mixed.is_3d())


class TestConflictResult(unittest.TestCase):
    """Test cases for the ConflictResult class."""
    
    def test_conflict_result_creation(self):
        """Test creation of conflict results."""
        # Default is clear with no conflicts
        result = ConflictResult()
        self.assertTrue(result.is_clear)
        self.assertEqual(len(result.conflict_details), 0)
    
    def test_add_conflict(self):
        """Test adding conflicts to the result."""
        result = ConflictResult()
        
        # Add a conflict
        result.add_conflict(
            location=(10.0, 20.0),
            time=datetime.now(),
            conflicting_drone_id="drone1",
            distance=5.0,
            min_safe_distance=10.0
        )
        
        # Status should be updated
        self.assertFalse(result.is_clear)
        self.assertEqual(len(result.conflict_details), 1)
        
        # Check conflict details
        conflict = result.conflict_details[0]
        self.assertEqual(conflict['location'], (10.0, 20.0))
        self.assertEqual(conflict['conflicting_drone_id'], "drone1")
        self.assertEqual(conflict['actual_distance'], 5.0)
        self.assertEqual(conflict['min_safe_distance'], 10.0)
    
    def test_string_representation(self):
        """Test string representation of conflict results."""
        # Clear result
        result = ConflictResult()
        self.assertIn("clear", str(result).lower())
        
        # Result with conflicts
        now = datetime.now()
        result.add_conflict(
            location=(10.0, 20.0),
            time=now,
            conflicting_drone_id="drone1",
            distance=5.0,
            min_safe_distance=10.0
        )
        
        # Add another conflict with a different drone
        result.add_conflict(
            location=(30.0, 40.0),
            time=now + timedelta(minutes=2),
            conflicting_drone_id="drone2",
            distance=8.0,
            min_safe_distance=10.0
        )
        
        # Check string representation
        result_str = str(result)
        self.assertIn("conflict detected", result_str.lower())
        self.assertIn("2 conflict", result_str.lower())  # Should mention 2 conflicts
        self.assertIn("2 drone", result_str.lower())     # Should mention 2 drones
        self.assertIn("drone1", result_str.lower())      # Should mention first drone


if __name__ == '__main__':
    unittest.main() 