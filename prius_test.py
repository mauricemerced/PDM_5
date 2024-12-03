import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from collections import deque
from heapq import heappush, heappop
import time


# Car parameters
L = 2.5  # Length of the car (in meters)
max_steering_angle = np.radians(30)  # Maximum steering angle (in radians)

def heuristic(x, y, goal):
    """Euclidean distance as a heuristic."""
    return np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)

# Lattice Planner Helper Functions
def is_within_bounds(x, y, grid_width=20, grid_height=20):
    """Check if the (x, y) position is within grid bounds."""
    return 0 <= x < grid_width and 0 <= y < grid_height

def is_collision_free(x, y):
    """Check if the given (x, y) position is free of obstacles using the environment."""
    # This is a placeholder 
    # Use environment's methods to check the current position in the simulation.
    return True

def car_model(x, y, theta, v, steering_angle, dt=0.1):
    """Car dynamics model (bicycle model)."""
    if steering_angle != 0:
        R = L / np.tan(steering_angle)  # Turning radius
        dtheta = v / R
    else:
        dtheta = 0 #go straigth

    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    
    return x + dx * dt, y + dy * dt, theta + dtheta * dt

# def lattice_planner(start, goal, env: UrdfEnv, max_steps=100, max_steering_angle=np.radians(30)):
    # """Lattice planner using BFS with Reeds-Shepp motion primitives."""
    # queue = deque([(start[0], start[1], start[2], 0, [])])  # (x, y, theta, cost, path)
    # visited = set()
    # visited.add((start[0], start[1], start[2]))
    
    # while queue:
    #     x, y, theta, cost, path = queue.popleft()
    #     print(x,y)
        
    #     # if goal[0] - 0.01 <= x <= goal[0] + 0.01 and goal[1] - 0.01 <= y <= goal[1] + 0.01:  # Goal reached
    #     if x == goal[0] and y == goal[1]:
    #         return path
        
    #     # Generate possible next steps using Reeds-Shepp-like motion primitives
    #     for steering_angle in np.linspace(-max_steering_angle, max_steering_angle, 3):  # 5 different steering angles
    #         new_x, new_y, new_theta = car_model(x, y, theta, 3, steering_angle)  # Assuming constant speed
    #         new_cost = cost + 1  # Simplified cost function (just number of steps)
            
    #         # Check if the new position is collision-free and not visited
    #         if is_collision_free(new_x, new_y, env) and (new_x, new_y, new_theta) not in visited:
    #             visited.add((new_x, new_y, new_theta))
    #             new_path = path + [(new_x, new_y, new_theta)]
                # queue.append((new_x, new_y, new_theta, new_cost, new_path))
                
                # if new_cost > max_steps:
                #     return None  # Return None if no solution is found within max_steps
    
    # return None  # Return None if goal is unreachable

def lattice_planner(start, goal, max_steps=100, max_steering_angle=np.radians(30)):
    """Lattice planner using A* with travel distance as cost."""
    queue = []
    heappush(queue, (0, start[0], start[1], start[2], 0, []))  # (priority, x, y, theta, cost_so_far, path)
    visited = set()
    visited.add((start[0], start[1], start[2]))
    
    while queue:
        _, x, y, theta, cost_so_far, path = heappop(queue)
        
        # Goal check with tolerance
        if np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2) < 0.1:
            return path
        
        for steering_angle in np.linspace(-max_steering_angle, max_steering_angle, 3):
            new_x, new_y, new_theta = car_model(x, y, theta, 3, steering_angle)  # Velocity fixed at 3 m/s
            step_distance = np.sqrt((new_x - x)**2 + (new_y - y)**2)  # Distance traveled in this step
            new_cost = cost_so_far + step_distance  # Accumulate distance
            
            if is_collision_free(new_x, new_y) and (new_x, new_y, new_theta) not in visited:
                visited.add((new_x, new_y, new_theta))
                new_path = path + [(new_x, new_y, new_theta)]
                priority = new_cost + heuristic(new_x, new_y, goal)  # A* priority
                
                heappush(queue, (priority, new_x, new_y, new_theta, new_cost, new_path))
                
                if len(new_path) > max_steps:
                    return None  # Fail if path exceeds maximum allowed steps
    
    return None  # No valid path found

# Main function to run the Prius
def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=0.3,
            wheel_radius=0.31265,
            wheel_distance=0.494,
            spawn_offset=np.array([-0.435, 0.0, 0.05]),
            actuated_wheels=['front_right_wheel_joint', 'front_left_wheel_joint', 'rear_right_wheel_joint', 'rear_left_wheel_joint'],
            steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
            facing_direction='-x'
        )
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render
    )


    pos0 = np.array([0, 0, 0])  # Initial position
    goal_position = np.array([5, 0, 0])  # Goal position
    ob = env.reset(pos=pos0) # set the car in the env

    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    # Plan path using lattice planner
    path = lattice_planner(pos0, goal_position)
    print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")


    if path is None:
        print("No valid path found.")
        return
    
    print(f"Initial observation : {ob}")
    ob = ob[0]
    history = []
    
    # Execute the path
    print("Executing path -------------------------------------------------------------------------------------------------------------")
    print("Path: ", path)
    print(len(path))

    
    
    goal = False
    while goal == False:
    # for step in range(n_steps):
        # Get current position from observation
        path_goal = path[0]
       
        current_state = ob['robot_0']['joint_state']['position']
        # Replan path
        # path = lattice_planner(current_state, goal_position)
        print(path_goal)
        print(current_state)
        if path is None:
            print("No valid path found. Stopping simulation.")
            goal = True
            break
        
        # Execute only the first step of the newly planned path
        if len(path) > 0:
            print("a")
            next_target = path[0]
            action = np.array([3.0, next_target[2]])  # Constant speed, follow steering angle to next point
        else:
            print("b")
            action = np.array([0.0, 0.0])  # Stop if no further path
        
        
        # while current_state[0] != path_goal[0] and current_state[1] != path_goal[1]:
        while np.sqrt((current_state[0] - path_goal[0])**2 + (current_state[1] - path_goal[1])**2) > 0.5:
            # Step the environment
            print(path_goal)
            print(current_state)
            print("Action: ", action)
            ob, *_ = env.step(action)
            history.append(ob)

        print(path)
        path = path[1:]  # Remove the first point from the path
         
        # Print feedback
        print(f"Step : Position = {ob['robot_0']['joint_state']['position']}")

        # Check if near goal
        if np.sqrt((goal_position[0] - current_state[0])**2 + (goal_position[1] - current_state[1])**2) < 0.1:
            print("Goal reached!")
            goal = True
            break
    
    time.sleep(15)
    env.close()
    return history

    # for i in range(n_steps):
    #     if path and goal == False:
    #         target_position = path[i] if i < len(path) else path[-1]
    #         print("Target position: ", target_position[2])
            
    #         action = np.array([3.0, target_position[2]])  # Use target theta as steering angle
            
    #         ob, *_ = env.step(action)
    #         history.append(ob)

    #         # path = lattice_planner(ob['robot_0']['joint_state']['position'], goal_position, env)

           
    #         # print("environment location   :   ",ob['robot_0']['joint_state']['position'])
    #         if np.sqrt((goal_position[0] - ob['robot_0']['joint_state']['position'][0])**2 + (goal_position[1] - ob['robot_0']['joint_state']['position'][1])**2) < 0.5:
    #             print("Goal reached!")
    #             print(target_position)
    #             print(((target_position[0] - ob['robot_0']['joint_state']['position'][0])**2 + (target_position[1] - ob['robot_0']['joint_state']['position'][1])**2)**0.5)
    #             goal = True
    #             break
    # env.close()
    # return history

if __name__ == "__main__":
    run_prius(render=True)
