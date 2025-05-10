"""
Tests for the visualization capabilities.
"""
import sys
import os
import unittest
from datetime import datetime, timedelta
import matplotlib.pyplot as plt

# Add the parent directory to system path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from deconfliction import Waypoint, Mission, ConflictResult
from deconfliction.visualization import MissionVisualizer


class TestVisualization(unittest.TestCase):
    """Test cases for the visualization module."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.start_time = datetime.now()
        
        # Create a simple mission
        self.mission = Mission(
            waypoints=[
                Waypoint(x=0, y=0, time=self.start_time),
                Waypoint(x=100, y=0, time=self.start_time + timedelta(minutes=5)),
                Waypoint(x=100, y=100, time=self.start_time + timedelta(minutes=10)),
            ],
            start_time=self.start_time,
            end_time=self.start_time + timedelta(minutes=10),
            drone_id="primary"
        )
        
        # Create a simulated flight
        self.simulated = Mission(
            waypoints=[
                Waypoint(x=0, y=50, time=self.start_time),
                Waypoint(x=100, y=50, time=self.start_time + timedelta(minutes=5)),
            ],
            start_time=self.start_time,
            end_time=self.start_time + timedelta(minutes=5),
            drone_id="sim1"
        )
        
        # Create a conflict result
        self.clear_result = ConflictResult()
        
        # Create a result with conflicts
        self.conflict_result = ConflictResult()
        self.conflict_result.add_conflict(
            location=(50.0, 25.0),
            time=self.start_time + timedelta(minutes=2, seconds=30),
            conflicting_drone_id="sim1",
            distance=5.0,
            min_safe_distance=10.0
        )
        
        # Create the visualizer
        self.visualizer = MissionVisualizer()
    
    def test_plot_mission(self):
        """Test plotting a single mission."""
        # Plot the mission
        ax = self.visualizer.plot_mission(self.mission)
        
        # Verify the plot was created
        self.assertIsInstance(ax, plt.Axes)
        
        # Check that the plot contains a line with waypoints
        lines = ax.get_lines()
        self.assertGreater(len(lines), 0)
        
        # Clean up
        plt.close()
    
    def test_plot_missions(self):
        """Test plotting multiple missions."""
        # Plot multiple missions
        fig = self.visualizer.plot_missions(
            [self.mission, self.simulated],
            primary_mission=self.mission
        )
        
        # Verify the plot was created
        self.assertIsInstance(fig, plt.Figure)
        
        # Check that the plot contains at least one axis
        self.assertEqual(len(fig.axes), 1)
        
        # Check that the axes contain lines for missions
        ax = fig.axes[0]
        lines = ax.get_lines()
        self.assertGreaterEqual(len(lines), 2)  # At least one per mission
        
        # Clean up
        plt.close(fig)
    
    def test_plot_mission_with_conflicts(self):
        """Test plotting a mission with conflicts."""
        # Plot missions with conflicts
        fig = self.visualizer.plot_mission_with_conflicts(
            self.mission,
            self.conflict_result,
            [self.simulated]
        )
        
        # Verify the plot was created
        self.assertIsInstance(fig, plt.Figure)
        
        # Check that the plot contains at least one axis
        self.assertEqual(len(fig.axes), 1)
        
        # Check for the conflict marker (at least one circle patch)
        ax = fig.axes[0]
        patches = ax.patches
        self.assertGreater(len(patches), 0)
        
        # Clean up
        plt.close(fig)
    
    def test_plot_timeline(self):
        """Test plotting a mission timeline."""
        # Plot timeline
        fig = self.visualizer.plot_timeline(
            [self.mission, self.simulated],
            self.clear_result
        )
        
        # Verify the plot was created
        self.assertIsInstance(fig, plt.Figure)
        
        # Check that the plot contains at least one axis
        self.assertEqual(len(fig.axes), 1)
        
        # Check that there are lines for the missions
        ax = fig.axes[0]
        lines = ax.get_lines()
        self.assertGreaterEqual(len(lines), 2)  # At least one per mission
        
        # Clean up
        plt.close(fig)
    
    def test_animate_missions(self):
        """Test creating an animation of missions."""
        # Create animation
        anim = self.visualizer.animate_missions(
            [self.mission, self.simulated],
            self.conflict_result
        )
        
        # Verify animation was created
        self.assertIsNotNone(anim)
        
        # Check that animation has appropriate content
        fig = plt.gcf()
        self.assertIsInstance(fig, plt.Figure)
        
        # Clean up
        plt.close(fig)


if __name__ == '__main__':
    unittest.main() 