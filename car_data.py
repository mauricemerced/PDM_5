import numpy as np
from urdfenvs.urdf_common.bicycle_model import BicycleModel


# Car parameters
L = 2.5  # Length of the car (in meters)
max_speed = 3.0
max_steering_angle = np.radians(30)  # Maximum steering angle (in radians)


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
