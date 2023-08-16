import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import numpy as np
from robot_type import RobotType
from course import Course, Path

class DriveSim:
    def __init__(self, cfg, path_pattern=0, obstacle_pattern=0):

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
                              point_interval=self.point_interval,
                              path_pattern=path_pattern,
                              obstacle_pattern=obstacle_pattern
                              )
        ## Generate Path
        self.inital_pose, self.goal, self.path, self.obstacle = course_generater.get_course()       
        
    def get_initial_pose(self) -> np.array:
        """
        Return the initial robot pose
        """
        return self.inital_pose
    
    def get_goal_pose(self) -> np.array:
        """
        Return the goal pose
        """
        return self.goal

    def get_obstacle_pose(self) -> np.array:
        """
        Return the obstacle pose
        """
        return self.obstacle
    
    def update(self, x, u, dt):
        """
        Update robot pose and check if it has reached goal.
        """
        
        ## translational movement
        ### x[0]: x value of the robot pose
        ### x[1]: y value of the robot pose
        
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
            
        ## Check reaching goal
        dist_to_goal = math.hypot(x[0] - self.goal[0], x[1] - self.goal[1])
        if dist_to_goal <= self.robot_radius:
            print("Goal!!")
            return x, True   
            
        return x, False
    
    def update_animation(self, x: np.array, obstacle: np.array, trajectory: np.array):
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-g")
        plt.plot(x[0], x[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xb")
        self._plot_obstacle(obstacle)
        
        self._plot_path(self.path)
        
        self._plot_robot(x[0], x[1], x[2])
        self._plot_arrow(x[0], x[1], x[2])
        
        plt.xlim(self.plot_x_min, self.plot_x_max)
        plt.ylim(self.plot_y_min, self.plot_y_max)
        plt.grid(True)
        plt.pause(0.0001)

    def _plot_robot(self, x: np.array, y: np.array, yaw: np.array):
        """
        Plot robot pose
        """
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
        
    def _plot_obstacle(self, obstacles: np.array):
        """
        Plot obstacle pose
        """
        for ob in obstacles:
            plt.gca().add_patch(Rectangle((ob[0]- ob[2]/2, ob[1]- ob[3]/2), ob[2], ob[3]))

    def _plot_arrow(self, x: np.array, y: np.array, yaw: np.array, length=0.5, width=0.1):
        """
        Plot obstacle pose
        """
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        plt.plot(x, y)

    def _plot_path(self, path: Path):
        plt.plot(path.left_bound[:, 0], path.left_bound[:, 1], linewidth=4, color="black")
        plt.plot(path.right_bound[:, 0], path.right_bound[:, 1], linewidth=4, color="black")
    
    