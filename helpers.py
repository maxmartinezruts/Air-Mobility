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
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

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
