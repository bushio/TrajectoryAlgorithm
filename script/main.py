"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
import matplotlib.pyplot as plt
import numpy as np
import argparse
from config import Config

from drive_sim import DriveSim
from dynamic_window_approach import DynamicWindowApproach

show_animation = True
def argparser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--course', help='course pattern', default=0, type=int)
    parser.add_argument('--ob', help='object pattern', default=0, type=int)
    args = parser.parse_args() 
    return args
    
def main(cfg):
    args = argparser()
    print(__file__ + " start!!")
    
    game = DriveSim(cfg, path_pattern=args.course, obstacle_pattern=args.ob)
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
    