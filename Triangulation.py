import sys
import cv2
import numpy as np
import time


def find_depth(right_point, left_point, frame_right, frame_left, baseline, alpha):

    # CONVERT FOCAL LENGTH f FROM [mm] TO [pixel]:
    height_right, width_right, depth_right = frame_right.shape
    height_left, width_left, depth_left = frame_left.shape

    if width_right == width_left:
        f_pixel = (width_right * 0.5) / np.tan(alpha * 0.5 * np.pi/180)

    else:
        print('Left and right camera frames do not have the same pixel width')
    #f_pixel=100
    x_right = right_point[0]
    x_left = left_point[0]
    y_left = left_point[1]

    # CALCULATE THE DISPARITY:
    disparity = abs(x_left-x_right )     #Displacement between left and right frames [pixels]

    # CALCULATE DEPTH z:
    zDepth = (baseline*f_pixel)/disparity
    # CALCULATE X and Y:
    x = (zDepth * x_left) / f_pixel
    y = (zDepth * y_left) / f_pixel
    return [x, y, zDepth]






