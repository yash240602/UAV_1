"""
Core functionality for UAV Strategic Deconfliction System.

This module implements the main conflict detection algorithms for checking
spatial and temporal conflicts between drone missions.
"""
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from datetime import datetime, timedelta
from .models import Waypoint, Mission, ConflictResult


class DeconflictionSystem:
    """Main deconfliction system for checking mission safety."""
    
    def __init__(self, simulated_flights: List[Mission], min_safe_distance: float = 50.0):
        """
        Initialize the deconfliction system.
        
        Args:
            simulated_flights: List of other drone missions in the airspace
            min_safe_distance: Minimum safe distance between drones (meters)
        """
        self.simulated_flights = simulated_flights
        self.min_safe_distance = min_safe_distance
    
    def check_mission(self, mission: Mission) -> ConflictResult:
        """
        Check if a mission is clear to fly.
        
        Args:
            mission: The primary mission to check
            
        Returns:
            ConflictResult with status and conflict details if found
        """
        result = ConflictResult()
        
        for simulated_flight in self.simulated_flights:
            # Skip if this is the same drone
            if simulated_flight.drone_id == mission.drone_id:
                continue
                
            # Check if the missions have overlapping time windows
            if not self._time_windows_overlap(mission, simulated_flight):
                continue
                
            # Check for conflicts between the flight paths
            conflicts = self._check_path_conflicts(mission, simulated_flight)
            
            # Add any conflicts to the result
            for conflict in conflicts:
                result.add_conflict(
                    location=conflict['location'],
                    time=conflict['time'],
                    conflicting_drone_id=simulated_flight.drone_id,
                    distance=conflict['distance'],
                    min_safe_distance=self.min_safe_distance
                )
        
        return result
    
    def visualize(self, mission: Mission, result: Optional[ConflictResult] = None, 
                 save_path: Optional[str] = None) -> Any:
        """
        Visualize a mission check result.
        
        Args:
            mission: The primary mission that was checked
            result: The conflict check result (if None, will run check_mission)
            save_path: Optional path to save the visualization
            
        Returns:
            The visualization figure
        """
        # Import here to avoid circular import
        from .visualization import MissionVisualizer
        
        visualizer = MissionVisualizer()
        
        # If no result provided, run the check
        if result is None:
            result = self.check_mission(mission)
        
        # Generate the visualization
        fig = visualizer.plot_mission_with_conflicts(
            mission, result, self.simulated_flights)
        
        # Save if requested
        if save_path:
            fig.savefig(save_path, dpi=300, bbox_inches='tight')
            
        return fig
    
    def visualize_timeline(self, mission: Mission, 
                          result: Optional[ConflictResult] = None,
                          save_path: Optional[str] = None) -> Any:
        """
        Visualize the mission timeline with conflicts.
        
        Args:
            mission: The primary mission to visualize
            result: Optional conflict result (if None, will run check_mission)
            save_path: Optional path to save the visualization
            
        Returns:
            The timeline figure
        """
        # Import here to avoid circular import
        from .visualization import MissionVisualizer
        
        visualizer = MissionVisualizer()
        
        # If no result provided, run the check
        if result is None:
            result = self.check_mission(mission)
        
        # Include the primary mission in the missions list
        all_missions = self.simulated_flights.copy()
        
        # Make sure the primary mission is not duplicated
        if not any(m.drone_id == mission.drone_id for m in all_missions):
            all_missions.append(mission)
        
        # Generate the timeline visualization
        fig = visualizer.plot_timeline(all_missions, result)
        
        # Save if requested
        if save_path:
            fig.savefig(save_path, dpi=300, bbox_inches='tight')
            
        return fig
    
    def animate_mission(self, mission: Mission, 
                       result: Optional[ConflictResult] = None,
                       save_path: Optional[str] = None,
                       interval: int = 100) -> Any:
        """
        Create an animation of the mission with other flights.
        
        Args:
            mission: The primary mission to animate
            result: Optional conflict result (if None, will run check_mission)
            save_path: Optional path to save the animation
            interval: Time between animation frames (milliseconds)
            
        Returns:
            The animation object
        """
        # Import here to avoid circular import
        from .visualization import MissionVisualizer
        
        visualizer = MissionVisualizer()
        
        # If no result provided, run the check
        if result is None:
            result = self.check_mission(mission)
        
        # Include the primary mission in the missions list
        all_missions = self.simulated_flights.copy()
        
        # Make sure the primary mission is not duplicated
        if not any(m.drone_id == mission.drone_id for m in all_missions):
            all_missions.append(mission)
        
        # Generate the animation
        anim = visualizer.animate_missions(
            all_missions, result, interval, save_path)
            
        return anim
    
    def _time_windows_overlap(self, mission1: Mission, mission2: Mission) -> bool:
        """Check if two missions have overlapping time windows."""
        return (mission1.start_time < mission2.end_time and
                mission2.start_time < mission1.end_time)
    
    def _check_path_conflicts(self, 
                             mission1: Mission, 
                             mission2: Mission) -> List[Dict[str, Any]]:
        """
        Check for conflicts between two mission paths.
        
        This is the core algorithm that checks for both spatial and temporal
        conflicts between mission path segments.
        """
        conflicts = []
        
        # Get the path segments for both missions
        segments1 = mission1.get_path_segments()
        segments2 = mission2.get_path_segments()
        
        # Check each segment combination for conflicts
        for i, (start1, end1) in enumerate(segments1):
            for j, (start2, end2) in enumerate(segments2):
                # First check if the time windows for these segments overlap
                if not self._segment_times_overlap(start1, end1, start2, end2):
                    continue
                
                # Check for spatial conflicts
                conflict_details = self._check_segment_conflict(
                    start1, end1, start2, end2)
                
                if conflict_details:
                    conflicts.append(conflict_details)
        
        return conflicts
    
    def _segment_times_overlap(self, 
                              start1: Waypoint, 
                              end1: Waypoint,
                              start2: Waypoint, 
                              end2: Waypoint) -> bool:
        """Check if the time windows for two path segments overlap."""
        if not all([start1.time, end1.time, start2.time, end2.time]):
            return True  # If any time is missing, assume they could overlap
        
        return (start1.time < end2.time and start2.time < end1.time)
    
    def _check_segment_conflict(self,
                               start1: Waypoint,
                               end1: Waypoint,
                               start2: Waypoint,
                               end2: Waypoint) -> Optional[Dict[str, Any]]:
        """
        Check if two path segments have a spatial conflict.
        
        Uses line segment distance calculation to find the minimum distance
        between the two path segments. If they are closer than the safe distance,
        a conflict is reported.
        
        Returns:
            Dict with conflict details or None if no conflict
        """
        # Convert waypoints to numpy arrays for vector calculations
        include_z = all(wp.z is not None for wp in [start1, end1, start2, end2])
        
        p1 = start1.coordinates(include_z)
        p2 = end1.coordinates(include_z)
        p3 = start2.coordinates(include_z)
        p4 = end2.coordinates(include_z)
        
        # Calculate minimum distance between line segments
        min_dist, point1, point2 = self._min_distance_segments(p1, p2, p3, p4)
        
        # Check if the distance is less than the minimum safe distance
        if min_dist < self.min_safe_distance:
            # Calculate the time of conflict
            conflict_time = self._interpolate_time(
                start1, end1, start2, end2, point1, point2)
            
            # Return conflict details
            return {
                'location': tuple(point1[:2]),  # Only x,y for location
                'time': conflict_time,
                'distance': min_dist,
            }
        
        return None
    
    def _min_distance_segments(self, 
                              p1: np.ndarray, 
                              p2: np.ndarray,
                              p3: np.ndarray, 
                              p4: np.ndarray) -> Tuple[float, np.ndarray, np.ndarray]:
        """
        Calculate the minimum distance between two line segments.
        
        Args:
            p1, p2: Start and end points of first segment
            p3, p4: Start and end points of second segment
            
        Returns:
            Tuple of (minimum_distance, point_on_segment1, point_on_segment2)
        """
        # Vector calculations
        v1 = p2 - p1  # Direction vector of segment 1
        v2 = p4 - p3  # Direction vector of segment 2
        w0 = p1 - p3  # Vector between segment start points
        
        # Parameters for calculations
        a = np.dot(v1, v1)  # Squared length of segment 1
        b = np.dot(v1, v2)
        c = np.dot(v2, v2)  # Squared length of segment 2
        d = np.dot(v1, w0)
        e = np.dot(v2, w0)
        
        # Calculate parameters for closest points
        denom = a * c - b * b
        
        # Handle parallel lines
        if abs(denom) < 1e-8:
            # Find closest point on segment 1 to start of segment 2
            t0 = d / a if a > 1e-8 else 0
            t0 = max(0, min(1, t0))
            pt1 = p1 + t0 * v1
            
            # Find closest point on segment 2 to this point
            t1 = np.dot(pt1 - p3, v2) / c if c > 1e-8 else 0
            t1 = max(0, min(1, t1))
            pt2 = p3 + t1 * v2
            
            return np.linalg.norm(pt1 - pt2), pt1, pt2
        
        # Calculate closest points
        s = (b * e - c * d) / denom
        t = (a * e - b * d) / denom
        
        # Clamp parameters to segment bounds
        s = max(0, min(1, s))
        t = max(0, min(1, t))
        
        # Calculate closest points on the segments
        pt1 = p1 + s * v1
        pt2 = p3 + t * v2
        
        # Return minimum distance and points
        return np.linalg.norm(pt1 - pt2), pt1, pt2
    
    def _interpolate_time(self,
                         start1: Waypoint,
                         end1: Waypoint,
                         start2: Waypoint,
                         end2: Waypoint,
                         point1: np.ndarray,
                         point2: np.ndarray) -> datetime:
        """
        Interpolate the time of conflict based on the conflict points.
        
        Uses linear interpolation along the path segments to estimate
        when the conflict occurs.
        """
        # If any time is None, use the earlier start time
        if None in [start1.time, end1.time, start2.time, end2.time]:
            return min(
                start1.time or datetime.now(),
                start2.time or datetime.now()
            )
        
        # Calculate segment vectors
        v1 = end1.coordinates() - start1.coordinates()
        v2 = end2.coordinates() - start2.coordinates()
        
        # Calculate position ratios along segments (0-1)
        dist1 = np.linalg.norm(v1)
        dist2 = np.linalg.norm(v2)
        
        if dist1 < 1e-8:  # If first segment is a point
            ratio1 = 0
        else:
            p1_vec = point1[:2] - start1.coordinates()
            ratio1 = min(1, max(0, np.dot(p1_vec, v1) / (dist1 * dist1)))
            
        if dist2 < 1e-8:  # If second segment is a point
            ratio2 = 0
        else:
            p2_vec = point2[:2] - start2.coordinates()
            ratio2 = min(1, max(0, np.dot(p2_vec, v2) / (dist2 * dist2)))
        
        # Interpolate times
        time1 = self._interpolate_datetime(start1.time, end1.time, ratio1)
        time2 = self._interpolate_datetime(start2.time, end2.time, ratio2)
        
        # Return the earlier time (when the first drone would arrive)
        return min(time1, time2)
    
    def _interpolate_datetime(self, 
                             start_time: datetime, 
                             end_time: datetime, 
                             ratio: float) -> datetime:
        """Linear interpolation between two datetimes."""
        delta = (end_time - start_time).total_seconds()
        seconds = delta * ratio
        return start_time + timedelta(seconds=seconds) 