import numpy as np
import costmap as cm
class Kitchen:
    def __init__(self):
        collision = True
        while collision:
            pos =np.random.randn(2)*2.4
            if not cm.intersects(pos) and np.linalg.norm(pos-np.array([0,0]))<6:
                collision = False
        self.pos = pos
