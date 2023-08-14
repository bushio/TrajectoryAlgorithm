import numpy as np
class Path:
    def __init__(self):
        right_bound = np.array([])
        left_bound = np.array([])
        


def rotate_transform(x, radian):
    r = np.array([[np.cos(radian), -np.sin(radian)], [np.sin(radian), np.cos(radian)]])
    x = x @ r 
    return x