"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
import numpy as np

class DynamicWindowApproach:
    def __init__(self, config):
        self.min_speed = config.min_speed
        self.max_speed = config.max_speed
        
        self.max_yaw_rate = config.max_yaw_rate
        self.max_yaw_rate = config.max_yaw_rate
       
       
        self.max_accel = config.max_accel
        self.max_delta_yaw_rate = config.max_delta_yaw_rate
        
        self.dt = config.dt
        
        self.v_resolution = config.v_resolution
        self.yaw_rate_resolution = config.yaw_rate_resolution
        
        
        self.obstacle_cost_gain = config.obstacle_cost_gain
        self.speed_cost_gain = config.speed_cost_gain
        self.to_goal_cost_gain = config.to_goal_cost_gain
        
        self.predict_time = config.predict_time
    
        self.robot_radius = config.robot_radius # [m]
        self.robot_width = config.robot_width  # [m]
        self.robot_length = config.robot_length  # [m]
        self.robot_stuck_flag_cons = config.robot_stuck_flag_cons
        
    def get_next_step(self, x, goal, obstacle):
        """
        Return next control and trajectory
        """
        
        # Calcuration dynamic window by self property
        motion_dw = self._calc_dynamic_window(x)
        
        u, trajectory = self.calc_control_and_trajectory(x, motion_dw, goal,  obstacle)
        return u, trajectory
    
    
    def _calc_dynamic_window(self, x):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw
    

    def calc_control_and_trajectory(self, x, dw, goal, ob):
        """
        calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self._calc_target_heading_cost(trajectory[-1], goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self._calc_obstacle_cost(trajectory, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -self.max_delta_yaw_rate
        return best_u, best_trajectory
    
    
    def predict_trajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self._motion(x, [v, y], self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt

        return trajectory


    def _motion(self, x, u, dt):
        """
        motion model
        """
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        return x


    def _calc_target_heading_cost(self, final_pose, target):
        """
            heading is a measure of progress towards the
            goal location. It is maximal if the robot moves directly towards the target.
        """
        dx = target[0] - final_pose[0]
        dy = target[1] - final_pose[1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - final_pose[2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost


    def _calc_obstacle_cost(self, trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= self.robot_length / 2
        right_check = local_ob[:, 1] <= self.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.robot_length / 2
        left_check = local_ob[:, 1] >= -self.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                        np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    