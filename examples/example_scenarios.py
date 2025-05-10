"""
Example scenarios for UAV Strategic Deconfliction System.

This script demonstrates the system with various scenarios:
1. A conflict-free mission
2. A mission with spatial and temporal conflicts
"""
import os
import sys
from datetime import datetime, timedelta
import matplotlib.pyplot as plt

# Add the parent directory to system path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from deconfliction import Waypoint, Mission, DeconflictionSystem

# Create output directory for visualizations
os.makedirs('output', exist_ok=True)

def create_simulated_flights():
    """Create a set of simulated flights for testing."""
    # Define a common start time
    start_time = datetime.now()
    
    # Create several simulated flights
    flights = []
    
    # Flight 1: Simple horizontal path
    flight1 = Mission(
        waypoints=[
            Waypoint(x=0, y=100, time=start_time),
            Waypoint(x=200, y=100, time=start_time + timedelta(minutes=2)),
            Waypoint(x=400, y=100, time=start_time + timedelta(minutes=4)),
        ],
        start_time=start_time,
        end_time=start_time + timedelta(minutes=4),
        drone_id="drone1"
    )
    flights.append(flight1)
    
    # Flight 2: Diagonal path
    flight2 = Mission(
        waypoints=[
            Waypoint(x=0, y=0, time=start_time + timedelta(minutes=1)),
            Waypoint(x=200, y=200, time=start_time + timedelta(minutes=3)),
            Waypoint(x=400, y=400, time=start_time + timedelta(minutes=5)),
        ],
        start_time=start_time + timedelta(minutes=1),
        end_time=start_time + timedelta(minutes=5),
        drone_id="drone2"
    )
    flights.append(flight2)
    
    # Flight 3: Vertical path
    flight3 = Mission(
        waypoints=[
            Waypoint(x=300, y=0, time=start_time + timedelta(minutes=2)),
            Waypoint(x=300, y=200, time=start_time + timedelta(minutes=4)),
            Waypoint(x=300, y=400, time=start_time + timedelta(minutes=6)),
        ],
        start_time=start_time + timedelta(minutes=2),
        end_time=start_time + timedelta(minutes=6),
        drone_id="drone3"
    )
    flights.append(flight3)
    
    # Flight 4: Complex path
    flight4 = Mission(
        waypoints=[
            Waypoint(x=100, y=300, time=start_time + timedelta(minutes=3)),
            Waypoint(x=150, y=250, time=start_time + timedelta(minutes=4)),
            Waypoint(x=200, y=300, time=start_time + timedelta(minutes=5)),
            Waypoint(x=250, y=250, time=start_time + timedelta(minutes=6)),
            Waypoint(x=300, y=300, time=start_time + timedelta(minutes=7)),
        ],
        start_time=start_time + timedelta(minutes=3),
        end_time=start_time + timedelta(minutes=7),
        drone_id="drone4"
    )
    flights.append(flight4)
    
    return flights, start_time

def scenario_1_conflict_free(simulated_flights, start_time):
    """Demonstrate a conflict-free mission scenario."""
    print("\n=== Scenario 1: Conflict-Free Mission ===")
    
    # Create a mission that doesn't conflict with any other flights
    mission = Mission(
        waypoints=[
            Waypoint(x=100, y=400, time=start_time),
            Waypoint(x=200, y=500, time=start_time + timedelta(minutes=2)),
            Waypoint(x=300, y=500, time=start_time + timedelta(minutes=4)),
            Waypoint(x=400, y=400, time=start_time + timedelta(minutes=6)),
        ],
        start_time=start_time,
        end_time=start_time + timedelta(minutes=6),
        drone_id="primary"
    )
    
    # Initialize the deconfliction system with simulated flights
    deconf_system = DeconflictionSystem(simulated_flights)
    
    # Check for conflicts
    result = deconf_system.check_mission(mission)
    
    # Display the result
    print(result)
    
    # Create visualizations
    print("Creating visualizations...")
    deconf_system.visualize(
        mission, result, save_path='output/scenario1_spatial.png')
    deconf_system.visualize_timeline(
        mission, result, save_path='output/scenario1_timeline.png')
    
    # Generate animation
    # This will display the animation in a matplotlib window
    # Uncomment to enable animation display
    # anim = deconf_system.animate_mission(mission, result)
    # plt.show()
    
    return mission, result

def scenario_2_with_conflict(simulated_flights, start_time):
    """Demonstrate a mission with conflicts."""
    print("\n=== Scenario 2: Mission With Conflicts ===")
    
    # Create a mission that conflicts with other flights
    mission = Mission(
        waypoints=[
            Waypoint(x=0, y=250, time=start_time + timedelta(minutes=1)),
            Waypoint(x=150, y=150, time=start_time + timedelta(minutes=3)),
            Waypoint(x=350, y=250, time=start_time + timedelta(minutes=5)),
        ],
        start_time=start_time + timedelta(minutes=1),
        end_time=start_time + timedelta(minutes=5),
        drone_id="primary"
    )
    
    # Initialize the deconfliction system with simulated flights
    deconf_system = DeconflictionSystem(simulated_flights)
    
    # Check for conflicts
    result = deconf_system.check_mission(mission)
    
    # Display the result
    print(result)
    
    if not result.is_clear:
        print(f"Found {len(result.conflict_details)} conflicts:")
        for i, conflict in enumerate(result.conflict_details):
            print(f"  Conflict {i+1}:")
            print(f"    Location: ({conflict['location'][0]:.1f}, {conflict['location'][1]:.1f})")
            print(f"    Time: {conflict['time']}")
            print(f"    With drone: {conflict['conflicting_drone_id']}")
            print(f"    Distance: {conflict['actual_distance']:.1f}m (minimum safe: {conflict['min_safe_distance']}m)")
    
    # Create visualizations
    print("Creating visualizations...")
    deconf_system.visualize(
        mission, result, save_path='output/scenario2_spatial.png')
    deconf_system.visualize_timeline(
        mission, result, save_path='output/scenario2_timeline.png')
    
    # Generate animation
    # This will display the animation in a matplotlib window
    # Uncomment to enable animation display
    # anim = deconf_system.animate_mission(mission, result)
    # plt.show()
    
    return mission, result

def scenario_3_altitude_separation(simulated_flights, start_time):
    """Demonstrate altitude-based separation (3D conflict detection)."""
    print("\n=== Scenario 3: Altitude-Based Separation ===")
    
    # Create a mission that would conflict in 2D but not in 3D
    mission = Mission(
        waypoints=[
            Waypoint(x=0, y=250, z=150, time=start_time + timedelta(minutes=1)),
            Waypoint(x=150, y=150, z=150, time=start_time + timedelta(minutes=3)),
            Waypoint(x=350, y=250, z=150, time=start_time + timedelta(minutes=5)),
        ],
        start_time=start_time + timedelta(minutes=1),
        end_time=start_time + timedelta(minutes=5),
        drone_id="primary"
    )
    
    # Convert simulated flights to 3D (with z=0)
    simulated_flights_3d = []
    for flight in simulated_flights:
        waypoints_3d = []
        for wp in flight.waypoints:
            waypoints_3d.append(Waypoint(x=wp.x, y=wp.y, z=0, time=wp.time))
        
        flight_3d = Mission(
            waypoints=waypoints_3d,
            start_time=flight.start_time,
            end_time=flight.end_time,
            drone_id=flight.drone_id
        )
        simulated_flights_3d.append(flight_3d)
    
    # Initialize the deconfliction system with 3D flights
    deconf_system = DeconflictionSystem(simulated_flights_3d)
    
    # Check for conflicts
    result = deconf_system.check_mission(mission)
    
    # Display the result
    print(result)
    
    print("Note: Mission is using altitude separation (z=150m) to avoid conflicts")
    
    # Create visualizations
    print("Creating visualizations...")
    deconf_system.visualize(
        mission, result, save_path='output/scenario3_spatial.png')
    deconf_system.visualize_timeline(
        mission, result, save_path='output/scenario3_timeline.png')
    
    return mission, result

def main():
    """Run all example scenarios."""
    print("UAV Strategic Deconfliction System - Example Scenarios")
    
    # Create simulated flights
    simulated_flights, start_time = create_simulated_flights()
    
    # Run the scenarios
    scenario_1_conflict_free(simulated_flights, start_time)
    scenario_2_with_conflict(simulated_flights, start_time)
    scenario_3_altitude_separation(simulated_flights, start_time)
    
    print("\nAll scenarios completed. Visualizations saved to 'output' directory.")

if __name__ == "__main__":
    main() 