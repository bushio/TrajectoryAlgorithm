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
                 obstacle_pattern = 1,
                 path_width = 5.0
                 ):
        self.path_pattern = path_pattern
        self.obstacle_pattern = obstacle_pattern
        self.y_min = ymin
        self.x_min = xmin
        self.y_max = ymax
        self.x_max = xmax
        self.center_line = (xmax - xmin) / 2.0
        
        self.path_length = ymax - ymin
        self.goal_dist = self.path_length - 5.0
        self.path_width = path_width
        
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

        path = Path(length = self.path_length, width = self.path_width, center_line = self.center_line)
        point_num = int(path.length * int(1 / self.point_interval))
        longitudinal_interval = float(path.length/ point_num)
        
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        initial_pose = np.array([path.center_line, 0.0, math.pi / 2.0, 0.0, 0.0])
        goal_pose = np.array([path.center_line, 0.0 + self.goal_dist])
        x1 = [path.center_line - path.width / 2.0 for i in range(point_num)]
        y1 = [self.y_min + (i * longitudinal_interval) for i in range(point_num)]

        x2 = [path.center_line + path.width / 2.0 for i in range(point_num)]
        y2 = [self.y_min + (i * longitudinal_interval) for i in range(point_num)]
        
        path.left_bound = np.array(list(zip(x1, y1)))
        path.right_bound = np.array(list(zip(x2, y2)))
        
        return initial_pose, goal_pose, path

    def _generate_obstacle(self):
        if self.obstacle_pattern ==1:
            """ pattern=1
            |  G  |
            |--   |  
            |     |
            |   --|
            |     |
            |  S  |
            """
            nearest_ob_y = 5.25
            ob_interval = 10.0
            ob_width = [1.5, 2.0, 2.5]
            ob = []
            for ii, w in enumerate(ob_width):
                if ii % 2 == 0:
                    x = self.path.right_bound[0, 0] - w / 2.0
                else:
                    x = self.path.left_bound[0, 0] + w / 2.0

                y = nearest_ob_y + ob_interval * ii
                ob.append([x, y, w, self.obstacle_height])
            ob = np.array(ob)

        else:
            """ pattern=0
            |  G  |
            |   --|  
            |     |
            |--   |
            |     |
            |  S  |
            """
            nearest_ob_y = 5.25
            ob_interval = 10.0
            ob_width = [1.5, 2.0, 2.5]
            ob = []
            for ii, w in enumerate(ob_width):
                if ii % 2 == 0:
                    x = self.path.left_bound[0, 0] + w / 2.0
                else:
                    x = self.path.right_bound[0, 0] - w / 2.0
                
                y = nearest_ob_y + ob_interval * ii
                ob.append([x, y, w, self.obstacle_height])
            ob = np.array(ob)

        return ob
    

class Path:
    def __init__(self, length, width, center_line):
        self.right_bound = np.array([])
        self.left_bound = np.array([])
        self.center_line = center_line
        self.width = width
        self.length = length