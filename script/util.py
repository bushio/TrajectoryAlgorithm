import numpy as np
class Path:
    def __init__(self, length, width, center_line):
        self.right_bound = np.array([])
        self.left_bound = np.array([])
        self.center_line = center_line
        self.width = width
        self.length = length
        

def rotate_transform(x, radian):
    r = np.array([[np.cos(radian), -np.sin(radian)], [np.sin(radian), np.cos(radian)]])
    x = x @ r 
    return x