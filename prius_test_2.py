import argparse
from motion_primitive.lattice import lattice_planner
from car_data import max_speed

def run_prius(environment_module, render=True):
    """
    Run the Prius simulation with the lattice planner and a specified environment.
    """
    # Dynamically load the environment from the given module
    env = environment_module.setup_environment()

    # Define start and goal positions
    start = (0, 0, 0)  # Starting position: (x, y, heading angle)
    goal = (10, 10, 0)  # Goal position: (x, y, heading angle)
    target_speed = max_speed  # Speed in m/s (from car_data.py)

    print("Starting lattice planner...")
    # Call the lattice planner with the environment
    path = lattice_planner(start, goal, target_speed, env)

    if path:
        print("Path found!")
        print("Path:", path)

        # Render the environment and simulate robot movement along the path
        if render:
            env.reset_robot_positions({"prius": start})
            for waypoint in path:
                env.step({"prius": waypoint})
            env.render()
    else:
        print("No valid path found.")

    return {"path": path, "goal_position": goal}


if __name__ == "__main__":
    # Parse command-line arguments to select the environment setup script
    parser = argparse.ArgumentParser(description="Run the Prius simulation with specified environment.")
    parser.add_argument(
        "--environment",
        type=str,
        default="environment_base",
        help="The environment setup module to use (default: environment_base)."
    )
    args = parser.parse_args()

    # Dynamically import the selected environment setup module
    try:
        environment_module = __import__(args.environment)
    except ImportError as e:
        print(f"Error importing environment module '{args.environment}': {e}")
        exit(1)

    # Run the simulation with the specified environment
    run_prius(environment_module)
