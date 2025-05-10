"""
Visualization tools for UAV Strategic Deconfliction System.

This module provides plotting and animation capabilities to visualize
drone missions, flight paths, and detected conflicts.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.animation as animation
from typing import List, Tuple, Dict, Optional, Any
from datetime import datetime, timedelta
import matplotlib.dates as mdates
from .models import Waypoint, Mission, ConflictResult


class MissionVisualizer:
    """Visualizer for drone missions and conflicts."""
    
    def __init__(self, figsize: Tuple[int, int] = (10, 8)):
        """
        Initialize the mission visualizer.
        
        Args:
            figsize: Size of the figure (width, height) in inches
        """
        self.figsize = figsize
        self.colors = plt.cm.tab10.colors
        
    def plot_mission(self, 
                    mission: Mission, 
                    ax: Optional[plt.Axes] = None,
                    color: Optional[str] = None,
                    label: Optional[str] = None,
                    show_waypoints: bool = True) -> plt.Axes:
        """
        Plot a single mission path on the given axes.
        
        Args:
            mission: The mission to plot
            ax: Optional matplotlib axes to plot on
            color: Color to use for the mission path
            label: Label for the mission in the legend
            show_waypoints: Whether to show waypoint markers
            
        Returns:
            The matplotlib axes with the plot
        """
        if ax is None:
            _, ax = plt.subplots(figsize=self.figsize)
            
        # Extract waypoint coordinates
        x_coords = [wp.x for wp in mission.waypoints]
        y_coords = [wp.y for wp in mission.waypoints]
        
        # Use drone_id as label if none provided
        if label is None:
            label = f"Drone {mission.drone_id}"
            
        # Plot the mission path
        ax.plot(x_coords, y_coords, 'o-', 
                color=color, 
                label=label,
                markerfacecolor='white' if show_waypoints else 'none',
                markeredgecolor=color if color else None,
                markersize=8 if show_waypoints else 0)
        
        # Add waypoint numbers if showing waypoints
        if show_waypoints:
            for i, (x, y) in enumerate(zip(x_coords, y_coords)):
                ax.text(x, y, f" {i+1}", fontsize=9, 
                        verticalalignment='center')
        
        return ax
    
    def plot_missions(self, 
                     missions: List[Mission], 
                     primary_mission: Optional[Mission] = None,
                     title: str = "UAV Missions") -> plt.Figure:
        """
        Plot multiple missions on a single figure.
        
        Args:
            missions: List of missions to plot
            primary_mission: Optional primary mission to highlight
            title: Title for the plot
            
        Returns:
            Matplotlib figure with the plot
        """
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Plot the simulated missions
        for i, mission in enumerate(missions):
            # Skip the primary mission, we'll plot it last for emphasis
            if primary_mission and mission.drone_id == primary_mission.drone_id:
                continue
                
            color = self.colors[i % len(self.colors)]
            self.plot_mission(mission, ax, color=color, show_waypoints=False)
        
        # Plot the primary mission last and with emphasis
        if primary_mission is not None:
            self.plot_mission(
                primary_mission, 
                ax, 
                color='red', 
                label=f"Primary - Drone {primary_mission.drone_id}",
                show_waypoints=True
            )
        
        # Add labels and legend
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_title(title)
        ax.legend(loc='upper right')
        ax.grid(True, linestyle='--', alpha=0.7)
        
        # Equal aspect ratio for spatial plots
        ax.set_aspect('equal')
        
        plt.tight_layout()
        return fig
    
    def plot_mission_with_conflicts(self, 
                                   primary_mission: Mission,
                                   conflict_result: ConflictResult,
                                   simulated_flights: List[Mission]) -> plt.Figure:
        """
        Plot a mission with any detected conflicts highlighted.
        
        Args:
            primary_mission: The primary mission to check
            conflict_result: Conflict detection result
            simulated_flights: Other flights in the airspace
            
        Returns:
            Matplotlib figure with the plot
        """
        # Create the base plot with all missions
        fig = self.plot_missions(
            simulated_flights, 
            primary_mission,
            title=f"Mission Conflicts - Drone {primary_mission.drone_id}"
        )
        
        ax = fig.axes[0]
        
        # If there are no conflicts, just return the plot
        if conflict_result.is_clear:
            ax.set_title(f"Mission is Clear - No Conflicts Detected")
            return fig
            
        # Plot each conflict location
        for conflict in conflict_result.conflict_details:
            location = conflict['location']
            distance = conflict['actual_distance']
            safe_dist = conflict['min_safe_distance']
            
            # Draw a circle at the conflict location
            conflict_circle = Circle(
                location, 
                safe_dist,
                fill=True,
                color='red',
                alpha=0.3
            )
            ax.add_patch(conflict_circle)
            
            # Add a text label
            ax.text(
                location[0], 
                location[1] + safe_dist + 10,
                f"Conflict: {distance:.1f}m\nMin safe: {safe_dist}m",
                horizontalalignment='center',
                bbox=dict(facecolor='white', alpha=0.7)
            )
        
        ax.set_title(f"CONFLICT DETECTED - Drone {primary_mission.drone_id}")
        return fig
    
    def plot_timeline(self, 
                     missions: List[Mission],
                     conflict_result: Optional[ConflictResult] = None) -> plt.Figure:
        """
        Plot a timeline of missions with optional conflict markers.
        
        Args:
            missions: List of missions to include in the timeline
            conflict_result: Optional conflict result to mark conflicts
            
        Returns:
            Matplotlib figure with the timeline
        """
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Time range for all missions
        all_times = []
        for mission in missions:
            all_times.extend([mission.start_time, mission.end_time])
            for wp in mission.waypoints:
                if wp.time:
                    all_times.append(wp.time)
        
        min_time = min(all_times)
        max_time = max(all_times)
        
        # Add some buffer to the time range
        time_buffer = (max_time - min_time) * 0.1
        plot_min_time = min_time - time_buffer
        plot_max_time = max_time + time_buffer
        
        # Plot each mission as a horizontal line
        y_positions = {}
        for i, mission in enumerate(missions):
            y_pos = len(missions) - i  # Reverse order so primary mission is at the bottom
            y_positions[mission.drone_id] = y_pos
            
            # Plot mission time window
            ax.plot(
                [mission.start_time, mission.end_time], 
                [y_pos, y_pos],
                'o-',
                linewidth=2,
                color=self.colors[i % len(self.colors)],
                label=f"Drone {mission.drone_id}"
            )
            
            # Plot waypoints as dots on the line
            for wp in mission.waypoints:
                if wp.time:
                    ax.plot(
                        wp.time, 
                        y_pos, 
                        'o',
                        markersize=8,
                        markerfacecolor='white',
                        markeredgecolor=self.colors[i % len(self.colors)]
                    )
        
        # Mark conflicts if provided
        if conflict_result and not conflict_result.is_clear:
            for conflict in conflict_result.conflict_details:
                conflict_time = conflict['time']
                drone_id = conflict['conflicting_drone_id']
                
                if drone_id in y_positions:
                    # Mark the conflict with a red X
                    ax.plot(
                        conflict_time,
                        y_positions[drone_id],
                        'rx',
                        markersize=12,
                        markeredgewidth=3
                    )
        
        # Format the axes
        ax.set_yticks(list(y_positions.values()))
        ax.set_yticklabels([f"Drone {drone_id}" for drone_id in y_positions.keys()])
        ax.set_xlim(plot_min_time, plot_max_time)
        ax.set_ylim(0.5, len(missions) + 0.5)
        
        # Format the time axis
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
        plt.xticks(rotation=45)
        
        ax.set_title("Mission Timeline")
        ax.set_xlabel("Time")
        ax.grid(True, axis='x', linestyle='--', alpha=0.7)
        
        plt.tight_layout()
        return fig
    
    def animate_missions(self, 
                        missions: List[Mission],
                        conflict_result: Optional[ConflictResult] = None,
                        interval: int = 100,
                        save_path: Optional[str] = None) -> animation.FuncAnimation:
        """
        Create an animation of drone missions over time.
        
        Args:
            missions: List of missions to animate
            conflict_result: Optional conflict detection result
            interval: Time interval between frames (milliseconds)
            save_path: Optional path to save the animation
            
        Returns:
            Matplotlib animation object
        """
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Get time range for all missions
        all_times = []
        for mission in missions:
            for wp in mission.waypoints:
                if wp.time:
                    all_times.append(wp.time)
        
        min_time = min(all_times)
        max_time = max(all_times)
        
        # Create time steps for animation
        time_delta = (max_time - min_time) / 100  # 100 frames
        time_steps = [min_time + i * time_delta for i in range(101)]
        
        # Get spatial bounds
        x_coords = []
        y_coords = []
        for mission in missions:
            for wp in mission.waypoints:
                x_coords.append(wp.x)
                y_coords.append(wp.y)
        
        x_min, x_max = min(x_coords) - 50, max(x_coords) + 50
        y_min, y_max = min(y_coords) - 50, max(y_coords) + 50
        
        # Store mission paths and current positions
        paths = []
        current_positions = []
        
        # Initialize paths
        for i, mission in enumerate(missions):
            color = self.colors[i % len(self.colors)]
            
            # Create empty line for the path
            line, = ax.plot([], [], '-', color=color, alpha=0.5)
            
            # Create marker for current position
            marker, = ax.plot([], [], 'o', color=color, 
                             markersize=10, label=f"Drone {mission.drone_id}")
            
            paths.append((mission, line))
            current_positions.append((mission, marker))
        
        # Add conflict markers if available
        conflict_markers = []
        if conflict_result and not conflict_result.is_clear:
            for conflict in conflict_result.conflict_details:
                circle = plt.Circle(
                    conflict['location'], 
                    conflict['min_safe_distance'],
                    color='red', 
                    alpha=0.3,
                    fill=True,
                    visible=False
                )
                ax.add_patch(circle)
                conflict_markers.append((conflict['time'], circle))
        
        # Set up the figure
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_title('UAV Mission Animation')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend(loc='upper right')
        
        # Time display
        time_text = ax.text(
            0.02, 0.02, '', transform=ax.transAxes,
            bbox=dict(facecolor='white', alpha=0.7)
        )
        
        def init():
            """Initialize the animation."""
            for _, line in paths:
                line.set_data([], [])
            for _, marker in current_positions:
                marker.set_data([], [])
            time_text.set_text('')
            
            for _, circle in conflict_markers:
                circle.set_visible(False)
                
            return [line for _, line in paths] + \
                   [marker for _, marker in current_positions] + \
                   [time_text] + \
                   [circle for _, circle in conflict_markers]
        
        def animate(frame_idx):
            """Update the animation for each frame."""
            current_time = time_steps[frame_idx]
            
            # Update each drone's position
            for i, (mission, line) in enumerate(paths):
                # Get path up to the current time
                x_path = []
                y_path = []
                
                current_x = None
                current_y = None
                
                for j in range(len(mission.waypoints)-1):
                    start_wp = mission.waypoints[j]
                    end_wp = mission.waypoints[j+1]
                    
                    # Always include waypoints before current time
                    if start_wp.time <= current_time:
                        x_path.append(start_wp.x)
                        y_path.append(start_wp.y)
                    
                    # If we're between these waypoints, interpolate position
                    if start_wp.time <= current_time and end_wp.time >= current_time:
                        # Calculate position through linear interpolation
                        if start_wp.time == end_wp.time:
                            ratio = 0
                        else:
                            ratio = ((current_time - start_wp.time).total_seconds() / 
                                    (end_wp.time - start_wp.time).total_seconds())
                        
                        current_x = start_wp.x + ratio * (end_wp.x - start_wp.x)
                        current_y = start_wp.y + ratio * (end_wp.y - start_wp.y)
                
                # Add the final waypoint if we've reached it
                if mission.waypoints[-1].time <= current_time:
                    x_path.append(mission.waypoints[-1].x)
                    y_path.append(mission.waypoints[-1].y)
                    current_x = mission.waypoints[-1].x
                    current_y = mission.waypoints[-1].y
                
                # Update the path line
                line.set_data(x_path, y_path)
                
                # Update the current position marker
                _, marker = current_positions[i]
                if current_x is not None and current_y is not None:
                    marker.set_data([current_x], [current_y])
                else:
                    marker.set_data([], [])
            
            # Update conflict markers
            for conflict_time, circle in conflict_markers:
                circle.set_visible(conflict_time <= current_time)
            
            # Update time display
            time_text.set_text(f'Time: {current_time.strftime("%H:%M:%S")}')
            
            return [line for _, line in paths] + \
                   [marker for _, marker in current_positions] + \
                   [time_text] + \
                   [circle for _, circle in conflict_markers]
        
        # Create the animation
        anim = animation.FuncAnimation(
            fig, animate, frames=len(time_steps),
            init_func=init, blit=True, interval=interval
        )
        
        # Save if requested
        if save_path:
            anim.save(save_path, writer='ffmpeg', fps=30)
        
        plt.tight_layout()
        return anim 