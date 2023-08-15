import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import numpy as np
from robot_type import RobotType
from util import Path
from course import Course

class DriveSim:
    def __init__(self, cfg):

        ## Robot parameter
        self.robot_radius = cfg.robot_radius # [m]
        self.robot_width = cfg.robot_width # [m]
        self.robot_length = cfg.robot_length  # [m]
        if cfg.robot_type =="circle":
            self.robot_type = RobotType.circle
        else:
            self.robot_type = RobotType.rectangle
        
        ## World parameter
        self.plot_y_min = cfg.plot_y_min
        self.plot_y_max = cfg.plot_y_max
        self.plot_x_min = cfg.plot_x_min
        self.plot_x_max = cfg.plot_x_max
        
        ## Path parameter
        self.point_interval = cfg.point_interval
        
        course_generater = Course(xmin=self.plot_x_min,
                              ymin=self.plot_y_min,
                              xmax=self.plot_x_max,
                              ymax=self.plot_y_max,
                              point_interval=self.point_interval
                              )
        ## Generate Path
        self.inital_pose, self.goal, self.path, self.obstacle = course_generater.get_course()       
        
    def get_initial_pose(self):
        return self.inital_pose
    
    def get_goal_pose(self):
        return self.goal

    def get_obstacle_pose(self):
        return self.obstacle
    
    def update(self, x, u, dt):
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]
            
        # check reaching goal
        dist_to_goal = math.hypot(x[0] - self.goal[0], x[1] - self.goal[1])
        if dist_to_goal <= self.robot_radius:
            print("Goal!!")
            return x, True   
            
        return x, False
    
    def update_animation(self, x, obstacle, trajectory):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-g")
        plt.plot(x[0], x[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xb")
        #plt.plot(obstacle[:, 0], obstacle[:, 1], "ok")
        self._plot_obstacle(obstacle)
        
        self._plot_path(self.path)
        
        self._plot_robot(x[0], x[1], x[2])
        self._plot_arrow(x[0], x[1], x[2])
        
        #plt.axis("equal")
        plt.xlim(self.plot_x_min, self.plot_x_max)
        plt.ylim(self.plot_y_min, self.plot_y_max)
        plt.grid(True)
        
        plt.pause(0.0001)

    def _plot_robot(self, x, y, yaw):  # pragma: no cover
        
        if self.robot_type == RobotType.rectangle:
            outline = np.array([[-self.robot_length / 2, self.robot_length / 2,
                                (self.robot_length / 2), -self.robot_length / 2,
                                -self.robot_length / 2],
                                [self.robot_width / 2, self.robot_width / 2,
                                - self.robot_width / 2, -self.robot_width / 2,
                                self.robot_width / 2]])
            Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                            [-math.sin(yaw), math.cos(yaw)]])
            outline = (outline.T.dot(Rot1)).T
            outline[0, :] += x
            outline[1, :] += y
            plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), "-k")
        elif self.robot_type == RobotType.circle:
            circle = plt.Circle((x, y), self.robot_radius, color="b")
            plt.gcf().gca().add_artist(circle)
            out_x, out_y = (np.array([x, y]) +
                            np.array([np.cos(yaw), np.sin(yaw)]) * self.robot_radius)
            plt.plot([x, out_x], [y, out_y], "-k")
        
    def _plot_obstacle(self, obstacles):
        for ob in obstacles:
            plt.gca().add_patch(Rectangle((ob[0]- ob[2]/2, ob[1]- ob[3]/2), ob[2], ob[3]))

    def _plot_arrow(self, x, y, yaw, length=0.5, width=0.1):
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        plt.plot(x, y)

    def _plot_path(self, path):
        plt.plot(path.left_bound[:, 0], path.left_bound[:, 1], linewidth=4, color="black")
        plt.plot(path.right_bound[:, 0], path.right_bound[:, 1], linewidth=4, color="black")

    def _generate_path(self, point_interval=0.2, road_pattern=0):
        
        
        course_angle = 90
        self.path_radian = np.radians(course_angle)

        if road_pattern == 0:
            path = Path(length = 30.0, width = 5.0, center_line = 7.5)
            point_num = int(path.length * int(1 / point_interval))
            longitudinal_interval = float(path.length/ point_num)
            
            # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            initial_pose = np.array([path.center_line, 0.0, math.pi / 2.0, 0.0, 0.0])
            goal_pose = np.array([path.center_line, 18])
            x1 = [path.center_line - path.width / 2.0 for i in range(point_num)]
            y1 = [self.plot_y_min + (i * longitudinal_interval) for i in range(point_num)]

            x2 = [path.center_line + path.width / 2.0 for i in range(point_num)]
            y2 = [self.plot_y_min + (i * longitudinal_interval) for i in range(point_num)]
            
            path.left_bound = np.array(list(zip(x1, y1)))
            path.right_bound = np.array(list(zip(x2, y2)))
            
        return initial_pose, goal_pose, path

    def _generate_obstacle(self):
        obstacle_width = 2.5
        obstacle_height = 0.5
        ob = np.array([[6.25, 5.25, obstacle_width, obstacle_height, self.path_radian],
                       [8.75, 10.25, obstacle_width, obstacle_height, self.path_radian],
                       [6.25, 15.25, obstacle_width, obstacle_height, self.path_radian]
                       ])
        return ob
    
    