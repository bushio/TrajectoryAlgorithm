import matplotlib.pyplot as plt
import math
import numpy as np
from robot_type import RobotType
from util import Path

class DriveCourse:
    def __init__(self, cfg):
        
        self.robot_radius = cfg.robot_radius # [m]
        self.robot_width = cfg.robot_width # [m]
        self.robot_length = cfg.robot_length  # [m]
        if cfg.robot_type =="circle":
            self.robot_type = RobotType.circle
        else:
            self.robot_type = RobotType.rectangle
        
        self.plot_y_min = -2
        self.plot_y_max = 20
        self.plot_x_min = -2
        self.plot_x_max = 20
    
        self.inital_pose, self.goal, self.path = self._generate_path(road_pattern=0) 
        self.obstacle = self._generate_obstacle()
        
    def get_initial_pose(self):
        return self.inital_pose
    
    def get_goal_pose(self):
        return self.goal

    def _generate_obstacle(self):
        ob = np.array([[-1, -1],
                        [0, 2],
                        [4.0, 2.0],
                        [5.0, 4.0],
                        [5.0, 5.0],
                        [5.0, 6.0],
                        [5.0, 9.0],
                        [8.0, 9.0],
                        [7.0, 9.0],
                        [8.0, 10.0],
                        [9.0, 11.0],
                        [12.0, 13.0],
                        [12.0, 12.0],
                        [15.0, 15.0],
                        [13.0, 13.0]
                        ])
        return ob
    

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
        plt.plot(obstacle[:, 0], obstacle[:, 1], "ok")
        
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
        

    def _plot_arrow(self, x, y, yaw, length=0.5, width=0.1):
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        plt.plot(x, y)


    def _generate_path(self, road_pattern=0):
        path = Path()
        if road_pattern == 0:
            point_num = 5
            road_center = 7.5
            road_width = 5.0
            road_length = 15.0
            longitudinal_interval = float(road_length/ point_num)
            
            # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
            initial_pose = np.array([road_center, 0.0, math.pi / 2.0, 0.0, 0.0])
            goal_pose = np.array([road_center, 10])
            x1 = [road_center - road_width / 2.0 for i in range(point_num)]
            y1 = [self.plot_y_min + (i * longitudinal_interval) for i in range(point_num)]

            x2 = [road_center + road_width / 2.0 for i in range(point_num)]
            y2 = [self.plot_y_min + (i * longitudinal_interval) for i in range(point_num)]
                        
            path.left_bound = np.array(list(zip(x1, y1)))
            path.right_bound = np.array(list(zip(x2, y2)))

        return initial_pose, goal_pose, path