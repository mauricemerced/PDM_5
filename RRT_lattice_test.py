import numpy as np
from car_data import robots, target_speed
from RRT_test import rrt_star_with_tree  # Import the RRT function from RRT_test.py
from lattice_test import run_prius, lattice_planner2, ride_prius2, plot_motion_trajectory

if __name__ == "__main__":
    # First, generate lattice trajectories with run_prius
    lattice_trajectories = run_prius(robots, render=False)
    print("Lattice trajectories generated.")

    # Define your start and goal states for RRT
    start_rrt = (2.0, 2.0, 0.0)
    goal_rrt = (18.0, 18.0, 0.0)

    # Run RRT to find a path
    path_rrt, tree = rrt_star_with_tree(start_rrt, goal_rrt, target_speed=target_speed)

    if path_rrt is None:
        print("No path found by RRT!")
    else:
        print("RRT path found!")
        print("RRT Path:", path_rrt)

        # Use the endpoint of the RRT path as the goal for lattice planning
        # Assuming the last node in path_rrt is the goal the RRT reached
        x, y, _ = path_rrt[-1]
        start = (0, 0, 0)
        goal = (x, y)

        print("Planning path with lattice_planner2...")
        path_lattice, lattice = lattice_planner2(start, goal, lattice_trajectories)

        if path_lattice is None:
            print("No valid lattice path found.")
        else:
            print("Lattice path found!")
            print("Lattice Path:", path_lattice)
            print("Lattice (steering angles):", lattice)

            # Ride along the lattice path
            history, ends = ride_prius2(start, path_lattice)

            # Visualize the final motion
            plot_motion_trajectory(goal, history, path_lattice)
