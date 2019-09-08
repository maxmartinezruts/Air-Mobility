#-------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Describes state of the simulation, used by other scripts to facilitate acces to game objects
#-------------------------------#

# All environment parameters

# Time of simulation
class Environment:
    def __init__(self):
        self.tstep = 0

        # Objects located in environment
        self.drones = []
        self.kitchens = []
        self.orders = []

        # Grid refinement (Spatial grid is N * N)
        self.n = 100
        self.m = 20

        # List containing all drones positions at all time
        self.pos_drones = []
        self.dir_drones = []
        self.paths = []

env = Environment()
