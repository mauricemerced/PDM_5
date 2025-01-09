import matplotlib.pyplot as plt
import pybullet as p
from RRT_star import rrt_star_with_tree, plot_tree
import numpy as np
import time
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from car_data import target_speed, max_steering_angle
import math
from environment_setup import load_environment


def ride_prius_rrt(loaded_env, start, path):
    ob, *_ = loaded_env.reset(pos=np.array(start))
    print("Initial observation:", ob)

    current_state = ob['robot_0']['joint_state']['position']
    print(f"Starting position: {current_state}")
    history = []
    action = np.array([0.0, 0.0])

    # Initial camera setup parameters
    cameraDistance = 5.0  # Distance behind the car
    cameraYaw = 0         # Will be updated based on heading
    cameraPitch = -30      # Slight angle above the car
    cameraHeight = 0.5    # Height of the camera above the ground

    for next_target in path:
        target_reached = False
        while not target_reached:
            dx = next_target[0] - current_state[0]
            dy = next_target[1] - current_state[1]
            distance = np.sqrt(dx**2 + dy**2)

            # Accelerate gradually up to target_speed
            if action[0] < target_speed:
                action[0] += 0.1

            # Compute desired heading, apply steering control (clamped to max steering angle)
            desired_theta = math.atan2(dy, dx)
            current_heading = current_state[2]
            heading_error = (desired_theta - current_heading + math.pi) % (2 * math.pi) - math.pi
            action[1] = np.clip(heading_error, -max_steering_angle, max_steering_angle)

            # Step the environment
            ob, *_ = loaded_env.step(action)
            history.append(ob)
            current_state = ob['robot_0']['joint_state']['position']
            
            # Debug: Print current state, target, and action
            # print(f"Current State: {current_state}")
            # print(f"Moving towards target: {next_target}")
            # print(f"Distance to target: {distance}")
            # print(f"Heading error: {heading_error}")
            # print(f"Action applied: {action}")

            # Update camera after moving
            # Place camera behind the car based on current heading
            cam_x = current_state[0] - cameraDistance * math.cos(current_heading)
            cam_y = current_state[1] - cameraDistance * math.sin(current_heading)
            cam_z = cameraHeight

            # Convert the current_heading to degrees for yaw
            cameraYaw = math.degrees(current_heading) - 90  # camera faces the car from behind
            p.resetDebugVisualizerCamera(cameraDistance=cameraDistance,
                                         cameraYaw=cameraYaw,
                                         cameraPitch=cameraPitch,
                                         cameraTargetPosition=[current_state[0], current_state[1], cameraHeight])

            # Slow down the simulation for better visualization
            time.sleep(0.05)

            if distance < 0.5:
                target_reached = True

        print(f"Reached target: {next_target}")

    print("Goal reached!")
    print("Closing environment...")
    loaded_env.close()
    history_ends = 0
    return history, history_ends

if __name__ == "__main__":
    # Test environment selection
    selected_env = "static2"  # Change this to 'basic', 'static', 'static2' 'narrow', or 'dynamic'
    print(f"Testing {selected_env} environment...")
    env, has_obstacles, obstacle_dict, start_rrt, goal_rrt = load_environment(selected_env)

    # Run RRT* to find a path
    path_rrt, tree = rrt_star_with_tree(env,has_obstacles, obstacle_dict, start_rrt, goal_rrt, target_speed=target_speed, grid_width=30, grid_height=30, max_iter=10000)

    if path_rrt is None:
        print("No path found by RRT!")
    else:
        print("RRT path found!")
        print("RRT Path:", path_rrt)

        # Ride along the RRT* path directly
        history, ends = ride_prius_rrt(env, start_rrt, path_rrt)

        plot_tree(path_rrt, tree, obstacle_dict, start_rrt, goal_rrt, grid_width=30, grid_height=30)

        # If you have a plotting function, you can plot this trajectory
        # Or simply watch it in the simulation window.

        # Plot trajectory (if needed)
        trajectory = [obs['robot_0']['joint_state']['position'][:2] for obs in history]
        plt.plot(*zip(*path_rrt), 'g--', label="Planned Path")
        plt.plot(*zip(*trajectory), 'b-', label="Car Path")
        plt.legend()
        plt.show()
