import numpy as np
import math

class Course:
    def __init__(self, 
                 xmin=0, 
                 ymin=0, 
                 xmax=100,
                 ymax=100, 
                 point_interval=0.2, 
                 course_angle=90,
                 path_pattern=0,
                 obstacle_pattern = 1
                 ):
        self.path_pattern = path_pattern
        self.obstacle_pattern = obstacle_pattern
        self.y_min = ymin
        self.x_min = xmin
        self.y_max = ymax
        self.x_max = xmax
        
        self.obstacle_width = 2.5
        self.obstacle_height = 0.5
        
        self.point_interval = point_interval
        self.course_angle = course_angle
        self.path_radian = np.radians(self.course_angle)
        
        ## Generate course when the instance is generated.
        self.initial_pose, self.goal_pose, self.path = self._generate_path()
        self.obstacles = self._generate_obstacle()
        
    def get_course(self):
        return self.initial_pose, self.goal_pose, self.path, self.obstacles
        
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
            """ pattern=0
            |  G  |
            |   --|  
            |     |
            |--   |
            |     |
            |  S  |
            """
            left_ob_x = self.path.center_line - self.path.width / 4
            right_ob_x = self.path.center_line + self.path.width / 4
            nearest_ob_y = 5.25
            ob_interval = 7.5
            ob = np.array([[left_ob_x, nearest_ob_y, self.obstacle_width, self.obstacle_height],
                        [right_ob_x, nearest_ob_y + ob_interval, self.obstacle_width, self.obstacle_height]
                        ])

        elif self.obstacle_pattern ==1:
            """ pattern=1
            |  G  |
            |--   |  
            |     |
            |   --|
            |     |
            |  S  |
            """
            left_ob_x = self.path.center_line - self.path.width / 4
            right_ob_x = self.path.center_line + self.path.width / 4
            nearest_ob_y = 5.25
            ob_interval = 7.5
            ob = np.array([[right_ob_x, nearest_ob_y, self.obstacle_width, self.obstacle_height],
                        [left_ob_x, nearest_ob_y + ob_interval, self.obstacle_width, self.obstacle_height]
                        ])
        else:
            """ pattern=1
            |  G  |
            |--   |  
            |     |
            |   --|
            |     |
            |--   |
            |     |
            |  S  |
            """
            left_ob_x = self.path.center_line - self.path.width / 4
            right_ob_x = self.path.center_line + self.path.width / 4
            nearest_ob_y = 5.25
            ob_interval = 5
            ob = np.array([[left_ob_x, nearest_ob_y, self.obstacle_width, self.obstacle_height],
                        [right_ob_x, nearest_ob_y + ob_interval, self.obstacle_width, self.obstacle_height],
                        [left_ob_x, nearest_ob_y + ob_interval * 2, self.obstacle_width, self.obstacle_height]
                        ])
        return ob
    

class Path:
    def __init__(self, length, width, center_line):
        self.right_bound = np.array([])
        self.left_bound = np.array([])
        self.center_line = center_line
        self.width = width
        self.length = length