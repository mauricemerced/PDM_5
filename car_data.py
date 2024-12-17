import numpy as np
from urdfenvs.urdf_common.bicycle_model import BicycleModel


# Car parameters
L = 2.5  # Length of the car (in meters)

max_steering_angle = np.radians(25)  # Maximum steering angle (in radians)

target_speed = 5.0  # Fixed target speed 5m/s ~ 20km/h
n_points = 5

robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=0.3,
            wheel_radius=0.31265,
            wheel_distance=0.494,
            spawn_offset=np.array([-0.435, 0.0, 0.05]),
            actuated_wheels=['rear_right_wheel_joint', 'rear_left_wheel_joint'],
            steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
            facing_direction='-x'
        )
    ]


def car_model(x, y, theta, v, steering_angle, dt=0.5):
    """Car dynamics model (bicycle model)."""
    if steering_angle != 0:
        # R = L / np.tan(steering_angle)  # Turning radius
        # dtheta = v / R
        dtheta = v / L * np.tan(steering_angle)
    else:
        dtheta = 0 #go straigth

    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    
    return x + dx * dt, y + dy * dt, theta + dtheta * dt
