import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv

import time
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

from analysis import plot_trajectory, plot_lattice_trajectories,plot_motion_trajectory
from motion_primitive.lattice import lattice_planner, lattice_planner2
from car_data import L, max_steering_angle, robots, target_speed

import itertools

  

def run_prius(robot , render=True):
    DT = 0.005
    n_seconds=1.


    env = UrdfEnv(dt=0.005, robots=robots, render=False)
   
    action = np.array([1., 0])
  
    pos0 = np.array([0, 0, 0])
    ob, *_  = env.reset(pos=pos0)

    points = []

    steering_angles = [-25, -12.5, 0, 12.5, 25]  # Target steering angles
    lattice_trajectories = []

    for target_angle in steering_angles:
        print(target_angle, " ---------------------------------------------------------------")
        speed = ob['robot_0']['joint_state']['forward_velocity'][0]#1
        action = np.array([1., 0])
        trajectory = []
        ob, *_  = env.reset(pos=pos0)  # Reset environment for each steering angle

        end_time: Optional[float] = None
        state: int = 1

        for step in itertools.count(1):
            ob, *_ = env.step(action)

            if state == 1:
                if ob['robot_0']['joint_state']['forward_velocity'][0] >= target_speed:
                    print("speed reached")
                    state = 2
                else:
                    action[0] += 0.05

            if state == 2:
                action[0] = target_speed
                if target_angle == -25 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(-25)):
                    action[1] = -1.25
                elif target_angle == -12.5 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(-12.5)):
                    action[1] = -1.25
                elif target_angle == 25 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(25)):
                    action[1] = 1.25
                elif target_angle == 12.5 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(12.5)):
                    action[1] = 1.25
                else:
                    action[1] = 0

                if abs(ob['robot_0']['joint_state']['steering']) >= abs(np.radians(target_angle)):
                    state = 3
                    end_time = step * DT + n_seconds

            if state == 3:
                action[0] = target_speed
                action[1] = 0.  # Stop steering
                trajectory.append(ob)

                if step * DT > end_time:
                    break
        
        
        lattice_trajectories.append((target_angle, trajectory))
    

    env.close()
    return lattice_trajectories



def ride_prius(robot, act, start, goal):
    n_seconds=1
    DT = 0.005  # Simulation timestep
    env = UrdfEnv(dt=DT, robots=robot, render=True)
    pos0 = start  # Initial position
    goal_position = goal  # Goal position
    ob = env.reset(pos=pos0)

    current_state = ob[0]['robot_0']['joint_state']
    history = []
    history_ends = []

    # Add a marker for the goal position
    marker_dict = {
        "type": "sphere",
        "geometry": {
            "position": [float(goal_position[0]), float(goal_position[1]), float(1.0)],  # Cast to float
            "radius": float(0.09)  # Cast to float
        },
        "rgba": [float(0.3), float(0.5), float(0.6), float(1.0)],  # Cast to float
    }
    sphere_marker = SphereObstacle(name=f"sphere_marker_{0}", content_dict=marker_dict)
    env.add_obstacle(sphere_marker)

    for target_angle in act:
        print("ACTTTTTTTTTTTTTTTTTTTT")
        state = 1  # Initialize state
        speed = 1#current_state['forward_velocity'][0]  # Initial speed
        angle = 0  # Initial steering adjustment
        end_time = None

        speed = 1
        action = np.array([1., 0])
        trajectory = []
        

        end_time: Optional[float] = None
        state: int = 1

        end = []
        for step in itertools.count(1):
            ob, *_ = env.step(action)
            history.append(ob)
            end.append(ob)

            if state == 1:
                if ob['robot_0']['joint_state']['forward_velocity'][0] >= target_speed:
                    print("speed reached")
                    state = 2
                else:
                    action[0] += 0.05

            if state == 2:
                action[0] = target_speed
                if target_angle == -25 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(-25)):
                    action[1] = -1.25
                elif target_angle == -12.5 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(-12.5)):
                    action[1] = -1.25
                elif target_angle == 25 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(25)):
                    action[1] = 1.25
                elif target_angle == 12.5 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(12.5)):
                    action[1] = 1.25
                else:
                    action[1] = 0

                if abs(ob['robot_0']['joint_state']['steering']) >= abs(np.radians(target_angle)):
                    state = 3
                    end_time = step * DT + n_seconds

            if state == 3:
                action[0] = target_speed
                action[1] = 0.  # Stop steering
                trajectory.append(ob)

                if step * DT > end_time:
                    break

            if np.sqrt((goal[0] - ob['robot_0']['joint_state']['position'][0]) ** 2 + (goal[1] - ob['robot_0']['joint_state']['position'][1]) ** 2) < 0.5:
                print(np.sqrt((goal[0] - ob['robot_0']['joint_state']['position'][0]) ** 2 + (goal[1] - ob['robot_0']['joint_state']['position'][1]) ** 2))
                print("PATH REACED")
                break
        history_ends.append(end)
    time.sleep(2)
    env.close()
    return history, history_ends


def ride_prius2(start, path):
    env = UrdfEnv(dt=0.005, robots=robots, render=True)
    ob = env.reset(pos=start)

    current_state = ob[0]['robot_0']['joint_state']['position']
    history = []
    action = np.array([0.0, 0.0])
    for next_target in path:
        target_reached = False
          # [speed, steering angle]
        
        
        # Gradually accelerate and adjust steering
        while not target_reached:
            # Compute direction to target
            dx = next_target[0] - current_state[0]
            dy = next_target[1] - current_state[1]
            distance = np.sqrt(dx**2 + dy**2)
            
            # Adjust speed to target value
            if action[0] < target_speed:
                action[0] += 0.1  # Increment speed
            
            # Calculate desired steering angle
            desired_theta = np.arctan2(dy, dx)
            # action[1] = desired_theta - current_state[2]  # Adjust to face target
            # action[1] = np.clip(action[1], -max_steering_angle, max_steering_angle)
            # Limit steering angle
            current_heading = current_state[2]
            
            heading_error = (desired_theta - current_heading + np.pi) % (2 * np.pi) - np.pi
            action[1] = heading_error  # Steering angle adjustment

            # Limit steering angle
            action[1] = np.clip(action[1], -max_steering_angle, max_steering_angle)
            # if current_state[0] <-20 and current_state[1] > 17:
            #     print(action)

            ob, *_ = env.step(action)
            history.append(ob)
            
            current_state = ob['robot_0']['joint_state']['position']
            
            # Check if target is reached within a tolerance
            if distance < 0.5:  # 0.5 meters tolerance
                target_reached = True
            
        
        print(f"Reached target: {next_target}")
    
    print("Goal reached!")
    history_ends=0
    return history, history_ends


if __name__ == "__main__":

    lattice_trajectories = run_prius(robots, render=False)
    # run_prius(robots, render=True)
    # plot_lattice_trajectories(lattice_trajectories)
   

    x, y = -4, 0
    start = (0, 0, 0) 
    goal = (x, y)

    
    print("planning path..." )
    path, lattice = lattice_planner2(start, goal, lattice_trajectories)
   
    start = np.array([0, 0, 0]) 
    goal = np.array([x, y, 0]) 

    print(path, lattice)
    if path is None:
        print("No valid path found.")
    else:
        print("Path found!")
        print("Path:", path)
        print("Lattice (steering angles):", lattice)
        history, ends = ride_prius2(start, path)
        # history, ends = ride_prius2(robots, lattice, start, goal)
        
        # plot_trajectory(path, goal, history, ends)
        plot_motion_trajectory( goal, history, path)


