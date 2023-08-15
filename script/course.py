import numpy as np
import math
from util import Path

class Course:
    def __init__(self, 
                 xmin=0, 
                 ymin=0, 
                 xmax=100,
                 ymax=100, 
                 point_interval=0.2, 
                 course_angle=90,
                 path_pattern=0,
                 obstacle_pattern =0
                 ):
        self.path_pattern = path_pattern
        self.obstacle_pattern = obstacle_pattern
        self.y_min = ymin
        self.x_min = xmin
        self.y_max = ymax
        self.x_max = xmax
        self.point_interval = point_interval
        self.course_angle = course_angle
        self.path_radian = np.radians(self.course_angle)
    def get_course(self):
        initial_pose, goal_pose, path = self._generate_path()
        obstacles = self._generate_obstacle()
        return initial_pose, goal_pose, path, obstacles
        
    def _generate_path(self):
        if self.path_pattern == 0:
            path = Path(length = 30.0, width = 5.0, center_line = 7.5)
            point_num = int(path.length * int(1 / self.point_interval))
            longitudinal_interval = float(path.length/ point_num)
            
            # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            initial_pose = np.array([path.center_line, 0.0, math.pi / 2.0, 0.0, 0.0])
            goal_pose = np.array([path.center_line, 18])
            x1 = [path.center_line - path.width / 2.0 for i in range(point_num)]
            y1 = [self.y_min + (i * longitudinal_interval) for i in range(point_num)]

            x2 = [path.center_line + path.width / 2.0 for i in range(point_num)]
            y2 = [self.y_min + (i * longitudinal_interval) for i in range(point_num)]
            
            path.left_bound = np.array(list(zip(x1, y1)))
            path.right_bound = np.array(list(zip(x2, y2)))
        
        return initial_pose, goal_pose, path

    def _generate_obstacle(self):
        if self.obstacle_pattern ==0:
            obstacle_width = 2.5
            obstacle_height = 0.5
            ob = np.array([[6.25, 5.25, obstacle_width, obstacle_height, self.path_radian],
                        [8.75, 10.25, obstacle_width, obstacle_height, self.path_radian],
                        [6.25, 15.25, obstacle_width, obstacle_height, self.path_radian]
                        ])
        return ob