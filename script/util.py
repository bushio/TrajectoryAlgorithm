import numpy as np

def rotate_transform(x, radian):
    r = np.array([[np.cos(radian), -np.sin(radian)], [np.sin(radian), np.cos(radian)]])
    x = x @ r 
    return x