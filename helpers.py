# -------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Math helpers
# -------------------------------#

import numpy as np
import math
from environment import env

# Returns dot product of two vectors
def dotproduct(v1, v2):
    return sum((a * b) for a, b in zip(v1, v2))

# Returns lenght of a vector
def length(v):
    return math.sqrt(dotproduct(v, v))


# Returns angle between two vectors
def angle(v1, v2):


    res = dotproduct(v1, v2) / (length(v1) * length(v2))
    if -1<=res<=1:
        return math.acos(res)
    else:
        if res>1:
            return 0.0
        if res<-1:
            return math.pi
        return math.pi

# Returns coordinates given a cartesian position
def get_coor_by_pos(pos):
    x = int(round((pos[0] - (-8)) / (8 - (-8)) * ((env.n - 1))))
    y = int(round((pos[1] - (-8)) / (8 - (-8)) * ((env.n - 1))))
    if x < 0: x = 0
    if x >= env.n: x = env.n - 1
    if y < 0: y = 0
    if y >= env.n: y = env.n - 1
    coor = np.array([x, y], dtype=int)
    return coor
print(angle( np.array([-0.08040201,  0.08040201]) ,np.array([ 0.01414214, -0.01414214])))
