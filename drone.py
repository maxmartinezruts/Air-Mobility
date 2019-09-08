#-------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Includes all helpers related to the drone object
#-------------------------------#

import numpy as np
from environment import env
from point import Point
import math
import A_star as astar
import speed_mod as sp
import A_star_h as astarh
import time
import visual as vs
# Class Drone
class Drone:
    def __init__(self):
        # Set of states
        # 1. Available  - if drone is waiting in kitchen
        # 2. Carrying   - if drone is flying carrying an order
        # 3. Free       - if frone is flying not carrying an order

        # Cartesian position of the drone
        x_ = np.random.uniform(-8,8)
        y_ = np.random.uniform(-8,8)
        z_ = np.random.uniform( 0,20)
        self.pos = np.array([x_,y_,z_])

        # Start drone as free
        self.state = 'free'

        # Start drone with no orders
        self.order = None

        # Allocate random kitchen
        self.kitchen = np.random.choice(env.kitchens)

        self.dir = np.random.randn(2)

    def set_path(self):
        # Number of iterations to determine a trajectory
        self.recursion_depth = 8

        # Create path starting at drone position and ending at kitchen position
        path = [Point(self.pos), Point(self.kitchen.pos)]
        graph = astar.Graph()
        graph.prepare(astar.Node(self.pos[:2], None, graph, 0), astar.Node(self.kitchen.pos[:2], None, graph, math.inf))
        graph.search()
        # self.edges = graph.edges/

        # graph = Graph(path)
        #
        # # Find shorter paths
        # for i in range(self.recursion_depth):
        #     graph.search()
        #
        # # path = list(graph.path[1:])
        # path = graph.path
        # print(path)
        # pt = []
        # pos = np.array(list(self.pos))
        # i=0
        # while len(path) > 0:
        #     i+=1
        #     # VIRTUALLY Advance to target with a step lenght of 1/50
        #     step = (path[0].pos - pos) / (np.linalg.norm(path[0].pos - pos) + 0.000000000000001) *1/50
        #     pos += step
        #     if i%3==0:
        #         pt.append(Point(np.array(list(pos))))
        #
        #
        #     if np.linalg.norm(path[0].pos - pos) < (2 / 50):
        #         path.pop(0)

        pt = []
        for p in graph.path:
            pt.append(Point(np.array(list(p.pos))))
        graph = sp.Velpath(pt)
        for i in range(1):
            graph.search()

        # Convert path to list of nodes
        # print(graph.path)
        # graph = dt.Graph(graph.path, env.tstep)
        # graph.search()

        self.edges = graph.edges

        path = graph.get_path()

        # Optimize altitude:
        graph = astarh.Graph(path)
        graph.prepare(astarh.Node(None, graph, 0 ,self.pos[2]), astarh.Node(None, graph, len(path)-1, self.kitchen.pos[2], True))
        graph.search()
        time.sleep(1)

        print(self.edges[0].st, graph.path[0].pos, 'comparison')
        print(self.edges[0].en, graph.path[1].pos, 'comparison')

        print(self.edges[1].st, graph.path[1].pos, 'comparison')
        print(self.edges[1].en, graph.path[2].pos, 'comparison')


        for i in range(len(self.edges)):
            self.edges[i].st = graph.path[i].pos
            self.edges[i].en = graph.path[i+1].pos


        # Construct path with smaller steps
        env.paths.append(self.edges)
        sumle = []
        for edge in self.edges:
            sumle.append(edge.len/edge.vel)
        sumle = np.cumsum(np.array(sumle))
        t= 0
        i = 0
        print(sumle)
        path = {}
        genpos = np.array(self.edges[0].en)

        while t < sumle[-1]:
            if t > sumle[i]:
            #     print(t/50,sumle[i])
                i+=1
                genpos += self.edges[i].en - self.edges[i].st


            pos=genpos-(t-sumle[i])*(self.edges[i].st-self.edges[i].en)/self.edges[i].len*self.edges[i].vel
            path[t+env.tstep] = np.array(pos)
            t+=1
        self.post = path
        self.path = graph.path[1:]
        print('path:', self.post)



        # Add path to cost map in order to know the positions of the drones as a function of time
        self.add_path_to_costmap(list(self.edges), np.array(list(self.pos)))


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
                graph.prepare(astar.Node(self.pos, None, graph, 0), astar.Node(self.kitchen.pos, None, graph, math.inf))
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
        graph.prepare(astar.Node(self.pos, None, graph, 0), astar.Node(self.kitchen.pos, None, graph, math.inf))
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
        if env.tstep in self.post.keys():
            self.pos = self.post[env.tstep]
            if env.tstep-1 in self.post.keys():
                self.dir = self.post[env.tstep]-self.post[env.tstep-1]


    def add_path_to_costmap(self, edges, pos):
        # Add drone location in time to cost map
        for t in self.post.keys():
            if (np.linalg.norm(self.post[t] - self.kitchen.pos)>4/50):
                env.pos_drones[t+env.tstep].append(self.post[t])
                if t>0:
                    env.dir_drones[t+env.tstep].append( self.post[t] - self.post[t- 1])
                else:
                    env.dir_drones[t+env.tstep].append(np.array([1.,0.,0.0]))




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

