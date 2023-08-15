"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
import matplotlib.pyplot as plt
import numpy as np
from config import Config

from drive_sim import DriveSim
from dynamic_window_approach import DynamicWindowApproach

show_animation = True

def main(cfg):
    gx = 10.0
    gy=10.0
    
    print(__file__ + " start!!")
    
    game = DriveSim(cfg)
    path = game.path
    x = game.get_initial_pose()
    trajectory = np.array(x)
    goal = game.get_goal_pose()
    obstacle = game.get_obstacle_pose()
    dwa = DynamicWindowApproach(cfg)
    
    while True:
        u, predicted_trajectory = dwa.get_next_step(x, goal, obstacle, path)
        x, finish = game.update(x, u, cfg.dt)
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            game.update_animation(x, obstacle, predicted_trajectory)
        
        if finish:
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)
        plt.show()

if __name__ == '__main__':
    cfg = Config()
    main(cfg)
    