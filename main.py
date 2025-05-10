#!/usr/bin/env python
"""
UAV Strategic Deconfliction System.

This is the main entry point for the deconfliction system.
It provides a command-line interface to run various functions and examples.
"""
import argparse
import sys
import os
import matplotlib.pyplot as plt
from examples.example_scenarios import (
    scenario_1_conflict_free,
    scenario_2_with_conflict,
    scenario_3_altitude_separation,
    create_simulated_flights
)


def main():
    """Main entry point for the application."""
    parser = argparse.ArgumentParser(
        description='UAV Strategic Deconfliction System'
    )
    
    # Add command line arguments
    parser.add_argument(
        '--scenario',
        type=int,
        choices=[1, 2, 3],
        help='Run a specific example scenario (1, 2, or 3)'
    )
    
    parser.add_argument(
        '--animate',
        action='store_true',
        help='Show animations for the scenarios'
    )
    
    parser.add_argument(
        '--save-path',
        type=str,
        default='output',
        help='Directory to save visualizations (default: output)'
    )
    
    parser.add_argument(
        '--run-all',
        action='store_true',
        help='Run all example scenarios'
    )
    
    # Parse the arguments
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    os.makedirs(args.save_path, exist_ok=True)
    
    # Create the simulated flights (needed for all scenarios)
    simulated_flights, start_time = create_simulated_flights()
    
    # Run the specified scenario(s)
    if args.run_all or args.scenario is None:
        # Run all scenarios if no specific one requested
        print("Running all example scenarios")
        run_scenario(1, simulated_flights, start_time, args)
        run_scenario(2, simulated_flights, start_time, args)
        run_scenario(3, simulated_flights, start_time, args)
    else:
        # Run the specified scenario
        run_scenario(args.scenario, simulated_flights, start_time, args)
    
    print(f"\nVisualizations saved to '{args.save_path}' directory.")
    
    return 0


def run_scenario(scenario_num, simulated_flights, start_time, args):
    """Run a specific scenario with the given arguments."""
    from deconfliction import DeconflictionSystem
    
    print(f"\n=== Running Scenario {scenario_num} ===")
    
    # Run the appropriate scenario
    if scenario_num == 1:
        mission, result = scenario_1_conflict_free(simulated_flights, start_time)
    elif scenario_num == 2:
        mission, result = scenario_2_with_conflict(simulated_flights, start_time)
    elif scenario_num == 3:
        mission, result = scenario_3_altitude_separation(simulated_flights, start_time)
    else:
        print(f"Unknown scenario number: {scenario_num}")
        return
    
    # If animation requested, show it
    if args.animate:
        print("Generating animation...")
        system = DeconflictionSystem(simulated_flights)
        
        # Don't try to save the animation, just display it
        anim = system.animate_mission(mission, result, save_path=None)
        plt.show()


if __name__ == '__main__':
    sys.exit(main()) 