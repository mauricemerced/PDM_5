import numpy as np
from car_data import robots, target_speed
import time
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from car_data import robots, target_speed, max_steering_angle, car_model
import math
import matplotlib.pyplot as plt
import pybullet as p
from RRT_star import rrt_star_with_tree

def ride_prius_rrt(start, path):
    env = UrdfEnv(dt=0.005, robots=robots, render=True)
    ob, *_ = env.reset(pos=np.array(start))

    current_state = ob['robot_0']['joint_state']['position']
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

            # Compute desired heading
            desired_theta = math.atan2(dy, dx)
            current_heading = current_state[2]
            heading_error = (desired_theta - current_heading + math.pi) % (2 * math.pi) - math.pi
            action[1] = np.clip(heading_error, -max_steering_angle, max_steering_angle)

            ob, *_ = env.step(action)
            history.append(ob)
            current_state = ob['robot_0']['joint_state']['position']

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
    env.close()
    history_ends = 0
    return history, history_ends

if __name__ == "__main__":
    # Define start and goal states (x, y, theta)
    start_rrt = (2.0, 2.0, 0.0)
    goal_rrt = (18.0, 18.0, 0.0)

    # Run RRT* to find a path
    path_rrt, tree = rrt_star_with_tree(start_rrt, goal_rrt, target_speed=target_speed)

    if path_rrt is None:
        print("No path found by RRT!")
    else:
        print("RRT path found!")
        print("RRT Path:", path_rrt)

        # Ride along the RRT* path directly
        history, ends = ride_prius_rrt(start_rrt, path_rrt)

        # If you have a plotting function, you can plot this trajectory
        # Or simply watch it in the simulation window.
