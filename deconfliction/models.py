"""
Data models for UAV Strategic Deconfliction System.

This module defines the core data structures for representing drone missions,
waypoints, flight paths, and conflict detection results.
"""
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
import numpy as np
from datetime import datetime, timedelta


@dataclass
class Waypoint:
    """A single waypoint in 2D/3D space with optional timing information."""
    x: float
    y: float
    z: Optional[float] = None
    time: Optional[datetime] = None
    
    def coordinates(self, include_z: bool = False) -> np.ndarray:
        """Return the waypoint coordinates as a numpy array."""
        if include_z and self.z is not None:
            return np.array([self.x, self.y, self.z])
        return np.array([self.x, self.y])
    
    def distance_to(self, other: 'Waypoint', include_z: bool = False) -> float:
        """Calculate Euclidean distance to another waypoint."""
        return np.linalg.norm(
            self.coordinates(include_z) - other.coordinates(include_z)
        )


@dataclass
class Mission:
    """A complete drone mission with waypoints and time window."""
    waypoints: List[Waypoint]
    start_time: datetime
    end_time: datetime
    drone_id: str
    
    def __post_init__(self):
        """Validate mission data and set waypoint times if not provided."""
        if not self.waypoints:
            raise ValueError("Mission must have at least one waypoint")
        
        if self.start_time >= self.end_time:
            raise ValueError("Mission end time must be after start time")
        
        # If waypoints don't have times assigned, distribute them evenly
        if any(wp.time is None for wp in self.waypoints):
            self._assign_waypoint_times()
    
    def _assign_waypoint_times(self):
        """Assign times to waypoints based on even distribution in the mission window."""
        total_duration = (self.end_time - self.start_time).total_seconds()
        intervals = len(self.waypoints) - 1 if len(self.waypoints) > 1 else 1
        time_per_segment = total_duration / intervals
        
        # First waypoint is at start time
        self.waypoints[0].time = self.start_time
        
        # Distribute remaining waypoints evenly
        for i in range(1, len(self.waypoints)):
            seconds_offset = time_per_segment * i
            self.waypoints[i].time = self.start_time + timedelta(seconds=seconds_offset)
    
    def get_path_segments(self) -> List[Tuple[Waypoint, Waypoint]]:
        """Return the mission as a list of waypoint segment pairs."""
        return [(self.waypoints[i], self.waypoints[i+1]) 
                for i in range(len(self.waypoints)-1)]
    
    def is_3d(self) -> bool:
        """Determine if this is a 3D mission (has z coordinates)."""
        return all(wp.z is not None for wp in self.waypoints)


@dataclass
class ConflictResult:
    """Result of a conflict check, including detailed information if conflicts found."""
    is_clear: bool = True
    conflict_details: List[Dict[str, Any]] = field(default_factory=list)
    
    def add_conflict(self, location: Tuple[float, float], 
                     time: datetime, 
                     conflicting_drone_id: str,
                     distance: float,
                     min_safe_distance: float):
        """Add a conflict to the result."""
        self.is_clear = False
        self.conflict_details.append({
            'location': location,
            'time': time,
            'conflicting_drone_id': conflicting_drone_id,
            'actual_distance': distance,
            'min_safe_distance': min_safe_distance
        })
    
    def __str__(self) -> str:
        """String representation of the conflict result."""
        if self.is_clear:
            return "Mission is clear to fly - no conflicts detected."
        
        conflicts = len(self.conflict_details)
        drones = set(c['conflicting_drone_id'] for c in self.conflict_details)
        
        return (f"CONFLICT DETECTED: {conflicts} conflict(s) with {len(drones)} drone(s). "
                f"First conflict at {self.conflict_details[0]['time']} "
                f"with drone {self.conflict_details[0]['conflicting_drone_id']}.") 