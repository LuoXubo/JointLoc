"""
@Description :   Align the position to the global coordinate system.
@Author      :   Xubo Luo 
@Time        :   2024/02/17 16:56:38
"""

import os
import time
import cv2
import numpy as np

### Crater
R = np.array([[-2.05712553e-03, -9.99997845e-01, -2.78517010e-04],
                [9.99997729e-01, -2.05728034e-03, 5.56721881e-04],
                [-5.57293669e-04, -2.77371131e-04, 9.99999806e-01]])
t = np.array([97.24325644, -34.09808986, -9.40846656])
s = 0.049163886509760814

def align(pos):
    pos = np.array(pos)
    pos = pos * s
    pos = np.dot(R, pos) + t
    return pos

if __name__ == "__main__":
    pos = [0, 0, 0]
    pos = align(pos)
    print(pos)
    pass