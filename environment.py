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
        self.n = 200

        # List containing all drones positions at all time
        self.pos_drones = []

env = Environment()
