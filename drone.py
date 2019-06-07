import numpy as np
from environment import env
from point import Point
from graph import Graph
import A_star as astar

# Class Drone
class Drone:
    def __init__(self):
        # Set of states
        # 1. Available  - if drone is waiting in kitchen
        # 2. Carrying   - if drone is flying carrying an order
        # 3. Free       - if frone is flying not carrying an order

        # Cartesian position of the drone

        self.pos = np.random.uniform(-6,6,(2))

        # Start drone as free
        self.state = 'free'

        # Start drone with no orders
        self.order = None

        # Allocate random kitchen
        self.kitchen = np.random.choice(env.kitchens)

    def set_path(self):
        # Number of iterations to determine a trajectory
        self.recursion_depth = 7

        # Create path starting at drone position and ending at kitchen position
        path = [Point(self.pos), Point(self.kitchen.pos)]
        graph = astar.Graph()
        graph.prepare(astar.Node(self.pos, None, graph), astar.Node(self.kitchen.pos, None, graph))
        graph.search()

        # graph = Graph(path)
        #
        # # Find shorter paths
        # for i in range(self.recursion_depth):
        #     graph.search()


        self.path = graph.path[1:]

        # Add path to cost map in order to know the positions of the drones as a function of time
        self.add_path_to_costmap(list(graph.path), np.array(list(self.pos)))


    # Perform action
    def act(self):
        global orders

        # If carrying an order, move to order position, if arrive: drop and pick a kitchen to return to
        if self.state == 'carrying':
            # Advance to target
            self.navigate()

            # If arrived to customer
            if np.linalg.norm(self.order.pos - self.pos) < (4/50):

                # Set state to free, and order state to delivered
                self.state = 'free'
                self.order.state = 'delivered'

                # Chose a kitchen to return to
                self.kitchen = self.pick_kitchen(self.pos)

                # Create path starting at drone position and ending at kitchen position
                path = [Point(self.pos), Point(self.kitchen.pos)]

                graph = astar.Graph()
                graph.prepare(astar.Node(self.pos, None, graph), astar.Node(self.kitchen.pos, None, graph))
                graph.search()

                # graph = Graph(path)
                #
                # # Find shorter paths
                # for i in range(self.recursion_depth):
                #     graph.search()

                self.path = graph.path[1:]

                # Add path to cost map in order to know the positions of the drones as a function of time
                self.add_path_to_costmap(list(graph.path), np.array(list(self.pos)))

                # Delete order (already delivered)
                orders.remove(self.order)

        # If flying but not carrying
        if self.state == 'free':

            # Advance to kitchen
            self.navigate()

            # If arrived to kitchen, change state to available
            if np.linalg.norm(self.kitchen.pos - self.pos) < (4/50):
                self.state = 'available'

    # Returns true if drone is available
    def is_available(self):
        if self.state == 'available':
            return True
        else:
            return False

    # Take order
    def take_order(self, order):
        # Change state to carrying, and link the order
        self.order = order
        self.state = 'carrying'

        # Create path starting at drone position and ending at customer position
        path = [Point(self.pos), Point(order.pos)]
        graph = astar.Graph()
        graph.prepare(astar.Node(self.pos, None, graph), astar.Node(self.kitchen.pos, None, graph))
        graph.search()

        # graph = Graph(path)
        #
        # # Find shorter paths
        # for i in range(self.recursion_depth):
        #     graph.search()


        self.path = graph.path[1:]

        # Add path to cost map in order to know the positions of the drones as a function of time
        self.add_path_to_costmap(list(graph.path), np.array(list(self.pos)))



    def navigate(self):
        if len(self.path) > 0:
            # Advance to target with a step lenght of 1/50
            step = (self.path[0].pos - self.pos) / (np.linalg.norm(self.path[0].pos - self.pos)+0.000000000000001) / 50
            self.pos += step

            # If arrived to next point in path, pop point (so that path[0] will be the next point to pursue)
            if np.linalg.norm(self.path[0].pos - self.pos) < (4/50):
                self.path.pop(0)

    def add_path_to_costmap(self, path, pos):
        # Add drone location in time to cost map
        t = env.tstep

        while len(path) > 0:
            # VIRTUALLY Advance to target with a step lenght of 1/50
            step = (path[0].pos - pos) / (np.linalg.norm(path[0].pos - pos) + 0.000000000000001) / 50
            pos += step

            # Append drone positions to the list of other drone positions at this time
            env.pos_drones[t].append(np.array(list(pos)))

            if np.linalg.norm(path[0].pos - pos) < (4 / 50):
                path.pop(0)
            t += 1

    # Decide which kitchen to return to
    def pick_kitchen(self, pos):

        # First select best based on which kitchen is closest
        min_dist = 1000000
        for k in env.kitchens:
            if np.linalg.norm(pos - k.pos) < min_dist:
                min_dist = np.linalg.norm(pos - k.pos)
                best = k

        # But overwrite if a kitchen has few available drones
        for k in env.kitchens:
            n = 0
            # Count number of available drones
            for drone in env.drones:
                if drone.kitchen == k and drone.state == 'available':
                    n += 1

            # In case number of available drones is low in one kitchen, choose this one
            if n < 5:
                best = k
        return best

