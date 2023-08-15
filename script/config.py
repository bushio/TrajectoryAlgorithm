import math
import numpy as np
from robot_type import RobotType

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        
        # Resolution of translational motion
        self.v_resolution = 0.01  # [m/s]
        # Resolution of rotation motion
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        
        
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.5
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 1.0
        self.path_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        
        
        self.robot_type = "rectangle" #  "rectangle" or "circle"
        self.robot_radius = 1.0 # [m]
        self.robot_width = 1.0  # [m]
        self.robot_length = 2.0  # [m]