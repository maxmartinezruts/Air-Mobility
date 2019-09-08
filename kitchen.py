# -------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Includes all helpers related to the kitchen object
# -------------------------------#

import numpy as np
import costmap as cm

positions = [[2.34897116, 5.37814368,0], [4.48213918, -2.34546691,0], [2.2802122, -0.3632573,0], [-0.07741158, 1.84717171,0],
             [-3.44036818, - 0.26573057,0], [-1.45334161, 2.74929531,0], [1.44439719, -5.67881632,0],[0.37579733, 1.65503762,0],
             [ 0.20138089, -2.34668518,0],[2.68015335, 3.20869437,0],[ 2.32467124, -2.59386189,0],[-2.25475507,  3.50694518,0]]
class Kitchen:
    def __init__(self, i):
        collision = True
        while collision:
            pos =np.random.randn(2)*2.4
            if not cm.intersects(pos) and np.linalg.norm(pos-np.array([0,0]))<6:
                collision = False
        self.pos = np.array(positions[i])

