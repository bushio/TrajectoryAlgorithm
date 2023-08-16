"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import matplotlib.pyplot as plt
import numpy as np
import argparse
from config import Config

from drive_sim import DriveSim
from dynamic_window_approach import DynamicWindowApproach


def argparser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--course', help='course pattern', default=0, type=int)
    parser.add_argument('--ob', help='object pattern', default=0, type=int)
    parser.add_argument('--method', help='algorithm name', default="DWA")
    parser.add_argument('--no_animation', help='If set this option, animation is not displayed.', action='store_false')
    args = parser.parse_args() 
    return args


def main(cfg):
    args = argparser()
    show_animation = args.no_animation
    
    print(__file__ + " start!!")
    
    env = DriveSim(cfg, path_pattern=args.course, obstacle_pattern=args.ob)
    path = env.path
    x = env.get_initial_pose()
    trajectory = np.array(x)
    goal = env.get_goal_pose()
    obstacle = env.get_obstacle_pose()
    
    if args.method == "DWA":
        print("Use Dynamic Window Approach")
        predictor = DynamicWindowApproach(cfg)
    elif args.method == "RRT":
        print("RRT is not supported")
        exit()
    else:
        print("Use Dynamic Window Approach")
        predictor = DynamicWindowApproach(cfg)
    
    while True:
        u, predicted_trajectory = predictor.get_next_step(x, goal, obstacle, path)
        x, finish = env.update(x, u, cfg.dt)
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            env.update_animation(x, obstacle, predicted_trajectory)
        
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
    