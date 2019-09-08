#-------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Includes all helpers related to the order object
#-------------------------------#

import time
from environment import env
import numpy as np

class Order:
    def __init__(self):

        # Set of states
        # 1. waiting    - if order has not yet been taken by a drone
        # 2. moving     - if drone already took the order

        self.pos = np.random.randn(3)*4
        self.start_time = time.time()
        self.state = 'waiting'
        self.kitchen = np.random.choice(env.kitchens)