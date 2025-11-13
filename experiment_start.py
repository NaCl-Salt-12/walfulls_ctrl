#!/usr/bin/env python3
import json
import os
import subprocess
import sys
import tty
import termios
from pathlib import Path


class ExperimentCLI:
    def __init__(self):
        # File to store experiment history
        self.history_file = Path.home() / ".experiment_history.json"
        self.load_history()

    def clear_terminal(self):
        """Clears the terminal screen."""
        # Use 'cls' for Windows, 'clear' for Linux/macOS
        os.system("cls" if os.name == "nt" else "clear")

    def load_history(self):
        """Load experiment history from file"""
        if self.history_file.exists():
            with open(self.history_file, "r") as f:
                self.experiments = json.load(f)
        else:
            self.experiments = {}

    def save_history(self):
        """Save experiment history to file"""
        with open(self.history_file, "w") as f:
            json.dump(self.experiments, f, indent=2)

    def normalize_name(self, name):
        """Convert name to lowercase and replace spaces with underscores"""
        return name.lower().replace(" ", "_")

    def get_latest_experiments(self, count=10):
        """
        Returns a list of the names of the latest 'count' experiments.
        """
        all_names = list(self.experiments.keys())
        return all_names[-count:]

    def list_experiments(self, experiment_names=None):
        """
        List specified experiments or all previous experiments.
        If experiment_names is None, lists all.
        """
        if not self.experiments:
            print("No previous experiments found.")
            return

        if experiment_names is None:
            # List ALL experiments
            print("\nPrevious experiments (All):")
            experiments_to_list = self.experiments.items()
        else:
            # List only the specified subset (e.g., the latest 10)
            print("\nPrevious experiments (Latest 10):")
            # Create a dictionary of only the specified subset for listing
            subset = {
                name: self.experiments[name]
                for name in experiment_names
                if name in self.experiments
            }
            experiments_to_list = subset.items()

        exp_list = list(experiments_to_list)

        if not exp_list:
            print("No experiments to show.")
            return

        for i, (name, trial) in enumerate(exp_list, 1):
            print(f"  {i}. {name} (last trial: {trial:03d})")
        print()

    def print_naming_guidelines(self):
        """Print naming guidelines/conventions"""
        print(
            """
================================================================================
                    EXPERIMENT NAMING CONVENTIONS
================================================================================

FORMAT STRUCTURE
----------------
[task]_[environment]_[modifier1]_[modifier2]


CORE RULES
----------
1. Characters: Use only lowercase letters (a-z), numbers (0-9), and 
   underscores (_)

2. No timestamps: Dates and times are added automatically by the system

3. Order matters: Place descriptive elements before modifiers
   ✓ tumble_cinderblocks_50lbs_v2
   ✗ 50lbs_v2_tumble_cinderblocks


NAMING COMPONENTS
-----------------
Task (required): The primary action or test being performed
  Examples: tumble, climb, torque, speed

Environment (optional): The setting or context
  Examples: sandbag, cinderblock, obstacle_course, stairs, shin,

Modifiers (optional): Specific parameters or variations
  - Weight: 50lbs, 2kg
  - Versions: v1, v2, iter3
  - Conditions: high_speed, low_friction, partial_load


EXAMPLES
--------
• tumble_cinderblocks_50lbs
  Straightforward weighted tumbling test

• torque_shin_50lbs_v2
  Second version of shin torque test with added weight

• climb_stair_low_speed_25lbs
  Stair climb test with weight modifier


BEST PRACTICES
--------------
✓ Be specific but concise
  walk_uneven_terrain (not walking_test_on_terrain_that_is_uneven)

✓ Use consistent abbreviations
  Decide on 'lbs' vs 'pounds' and stick with it

✓ Version iteratively
  Use v1, v2 or iter1, iter2 for variations

✓ Group related experiments
  Use common prefixes for experiment families
"""
        )

    def get_experiment_name(self):
        """Prompt user for experiment name"""

        # Get the subset of experiments to show/select from (the latest 10)
        latest_experiments = self.get_latest_experiments(count=10)

        while True:
            # --- START: Clear the terminal on each loop iteration ---
            self.clear_terminal()
            # --- END: Clear the terminal on each loop iteration ---

            print("=" * 50)
            print("Experiment Naming Tool")
            print("=" * 50)
            print("\nPlease use standard naming conventions for experiments.")

            # Show only the LATEST 10 previous experiments if they exist (now inside the loop)
            if latest_experiments:
                self.list_experiments(experiment_names=latest_experiments)

            print("\nExperiment Name Options:")
            print("  - Type a **new** experiment name")
            print(
                f"  - Enter a number (1-{len(latest_experiments)}) to select from the **latest {len(latest_experiments)}** experiments"
            )
            print("  - Or type the exact name of an existing experiment")
            print("  - Type 'name' to see naming guidelines/conventions")
            print("  - Type 'list' to see **all** previous experiments")
            print("  - Type 'quit' to exit")

            user_input = input("\nExperiment name: ").strip()

            if user_input.lower() == "quit":
                self.clear_terminal()
                sys.exit(0)

            if user_input.lower() == "list":
                self.clear_terminal()  # Clear before listing all
                self.list_experiments(experiment_names=None)
                # Wait for user acknowledgment before continuing/re-clearing
                input("\nPress Enter to return to the selection menu...")
                continue

            if user_input.lower() == "name":
                self.clear_terminal()  # Clear before showing guidelines
                self.print_naming_guidelines()
                # Wait for user acknowledgment before continuing/re-clearing
                input("\nPress Enter to return to the selection menu...")
                continue

            # Check if input is a number (selecting from the latest 10 list)
            if user_input.isdigit():
                idx = int(user_input) - 1
                exp_list = latest_experiments
                if 0 <= idx < len(exp_list):
                    return exp_list[idx]
                else:
                    print(f"Invalid selection. Please choose 1-{len(exp_list)}")
                    input("\nPress Enter to try again...")
                    continue

            # Otherwise treat as new/existing experiment name
            if user_input:
                return self.normalize_name(user_input)
            else:
                print("Experiment name cannot be empty.")
                input("\nPress Enter to try again...")
                continue

    def get_trial_number(self, experiment_name):
        """Prompt user for trial number"""
        # Get suggested trial number
        if experiment_name in self.experiments:
            suggested_trial = self.experiments[experiment_name] + 1
        else:
            suggested_trial = 1

        # --- START: Clear the terminal before showing trial prompt ---
        self.clear_terminal()
        # --- END: Clear the terminal before showing trial prompt ---

        while True:
            user_input = input(
                f"\nExperiment: {experiment_name}\nTrial number [default: {suggested_trial:03d}]: "
            ).strip()

            # Use default if empty
            if not user_input:
                return suggested_trial

            # Validate input
            if user_input.isdigit():
                return int(user_input)
            else:
                print("Trial number must be a positive integer.")

    def run(self):
        """Main CLI loop"""

        # --- START: Initial clear (removed redundant header print) ---
        self.clear_terminal()
        # --- END: Initial clear ---

        # The header and experiment list display are now handled inside get_experiment_name()

        # Get experiment name
        experiment_name = self.get_experiment_name()

        # Get trial number
        trial_number = self.get_trial_number(experiment_name)

        # Create final experiment name
        final_name = f"{experiment_name}_{trial_number:03d}"

        # Update history
        if (
            experiment_name not in self.experiments
            or self.experiments[experiment_name] < trial_number
        ):
            self.experiments[experiment_name] = trial_number
        self.save_history()

        # Export to environment variable
        os.environ["EXPERIMENT_NAME"] = final_name

        # --- Clear the terminal one last time for the final results ---
        self.clear_terminal()
        # --- Final output display ---
        print("=" * 50)
        print("Launching Experiment")
        print("=" * 50)
        print(f"\nExperiment: {experiment_name}")
        print(f"Trial: {trial_number:03d}")
        print(f"Full name: {final_name}")
        print("\nStarting ROS2 launch...")
        print("=" * 50)
        print()

        # Launch ROS2 with the experiment name
        try:
            subprocess.run(
                [
                    "ros2",
                    "launch",
                    "./launch/variable_launch.py",
                    f"experiment_name:={final_name}",
                ]
            )
        except KeyboardInterrupt:
            print("\n\nLaunch interrupted by user.")
            sys.exit(0)
        except Exception as e:
            print(f"\n\nError launching ROS2: {e}")
            sys.exit(1)


if __name__ == "__main__":
    try:
        cli = ExperimentCLI()
        cli.run()
    except KeyboardInterrupt:
        print("\n\nOperation cancelled.")
        sys.exit(0)
