"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
import numpy as np

class DynamicWindowApproach:
    def __init__(self, config):
        ## Robot parameter
        self.min_speed = config.min_speed
        self.max_speed = config.max_speed
        self.max_yaw_rate = config.max_yaw_rate
        self.max_yaw_rate = config.max_yaw_rate
        self.max_accel = config.max_accel
        self.max_delta_yaw_rate = config.max_delta_yaw_rate
        
        self.dt = config.dt
        
        ## Search range
        self.v_resolution = config.v_resolution
        self.yaw_rate_resolution = config.yaw_rate_resolution
        
        ## Cost weights
        self.speed_cost_gain = config.speed_cost_gain
        self.to_goal_cost_gain = config.to_goal_cost_gain
        self.obstacle_cost_gain = config.obstacle_cost_gain
        self.path_cost_gain = config.path_cost_gain
        
        # If the distance between object and robot more than threshold , cost is ignored.
        self.ob_dist_threshold = config.ob_dist_threshold
        self.path_dist_threshold = config.path_dist_threshold
        
        ## Trajectory precition time 
        self.predict_time = config.predict_time
    
        ## Robot param
        self.robot_radius = config.robot_radius # [m]
        self.robot_width = config.robot_width  # [m]
        self.robot_length = config.robot_length  # [m]
        self.robot_stuck_flag_cons = config.robot_stuck_flag_cons
        
    def get_next_step(self, x, goal, obstacle, path):
        """
        Return next control and trajectory
        """
        
        # Calcuration dynamic window by self property
        motion_dw = self._calc_dynamic_window(x)
        
        u, trajectory = self.calc_control_and_trajectory(x, motion_dw, goal,  obstacle, path)
        return u, trajectory
    
    
    def _calc_dynamic_window(self, x):
        """
        Calculation dynamic window based on current state x
        """

        ## Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]

        ## Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw
    

    def calc_control_and_trajectory(self, x, dw, goal, ob, path):
        """
        Calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        ## Evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):
                
                ## Get next trajectory
                trajectory = self.predict_trajectory(x_init, v, y)
                
                ## Calculate cost
                to_goal_cost = self.to_goal_cost_gain * self._calc_target_heading_cost(trajectory[-1], goal)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self._calc_obstacle_cost(trajectory, ob)
                path_cost = self.path_cost_gain * self._calc_path_cost(trajectory, path)

                final_cost = to_goal_cost + speed_cost + ob_cost + path_cost

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
        Predict next trajectory
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
        Calculate motion model
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


    def _calc_obstacle_cost(self, trajectory, objects, dist_threshold=5.0, penalty=-1):        
        object_xy = objects[:, 0:2]
        object_wh = objects[:, 2:4]
        
        min_dist = float("Inf")
        if penalty == -1:
            penalty = float("Inf")
       
        ## Distance between object and trajectory points
        for tp in trajectory:

            ox = objects[:, 0]
            oy = objects[:, 1]
            dx = tp[0] - ox
            dy = tp[1] - oy
            dist = np.hypot(dx, dy)
            
            if min_dist < np.min(dist):
                min_dist = dist
            
            # Mask with distance
            mask = dist < dist_threshold
            object_xy_mask = object_xy[mask, :]
            object_wh_mask = object_wh[mask, :]
            
            ## Rotation matrix
            #yaw = tp[2]
            #rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            
            object_xy_new = object_xy_mask - tp[0:2]
            #object_xy_new = object_xy_new @ rot
            object_xy_new_upper_right = object_xy_new + object_wh_mask / 2
            object_xy_new_bottom_left = object_xy_new - object_wh_mask / 2
            
            right_check = object_xy_new_bottom_left[:, 0] >= self.robot_width / 2
            left_check = object_xy_new_upper_right[:, 0] <= -self.robot_width / 2
            top_check = object_xy_new_bottom_left[:, 1] >= self.robot_length / 2
            bottom_check = object_xy_new_upper_right[:, 1] <= -self.robot_length / 2
            
            check = np.prod(np.logical_or(np.logical_or(top_check, bottom_check), np.logical_or(right_check, left_check)))
            if not check:
                return penalty
        return 1.0 / min_dist  # OK
        

    def _calc_path_cost(self, trajectory, path, path_point_size=0.1, dist_threshold=5.0, penalty=-1):
        
        path_points = np.concatenate([path.left_bound, path.right_bound])
        object_xy = path_points[:, 0:2]
        points_size = np.array([path_point_size, path_point_size])

        if penalty == -1:
            penalty = float("Inf")
        min_dist = float("Inf")

        ## Distance between object and trajectory points
        for idx, tp in enumerate(trajectory):
            ox = path_points[:, 0]
            oy = path_points[:, 1]
            dx = tp[0] - ox
            dy = tp[1] - oy
            dist = np.hypot(dx, dy)
            
            if min_dist < np.min(dist):
                min_dist = dist
            
            # Mask with distance
            mask = dist < dist_threshold
            object_xy_mask = object_xy[mask, :]
            
            object_xy_new = object_xy_mask - tp[0:2]
            #object_xy_new = object_xy_new @ rot
            object_xy_new_upper_right = object_xy_new + points_size
            object_xy_new_bottom_left = object_xy_new - points_size
            
            right_check = object_xy_new_bottom_left[:, 0] >= self.robot_width / 2
            left_check = object_xy_new_upper_right[:, 0] <= -self.robot_width / 2
            top_check = object_xy_new_bottom_left[:, 1] >= self.robot_length / 2
            bottom_check = object_xy_new_upper_right[:, 1] <= -self.robot_length / 2
            
            check = np.prod(np.logical_or(np.logical_or(top_check, bottom_check), np.logical_or(right_check, left_check)))
            if not check:
                #print("collision")
                return penalty/(idx + 1)
            
        return 1.0 / min_dist  # OK
 
            

            
        


    