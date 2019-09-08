# import visual as vs
import costmap as cm
import numpy as np
import pygame
#-------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Own shortest path finding algorithm to optimize velocity along a found path to ensure geoclustered vectors
#-------------------------------#
from environment import env
import time
class Edge:
    def __init__(self, st, en, vel):
        self.st = st.pos
        self.en = en.pos
        self.vel = vel
        self.len = np.linalg.norm(self.en-self.st)
        self.stnode = st
        self.ennode = en


class Velpath:
    def __init__(self, path):
        self.t_ini = env.tstep
        self.vels = np.linspace(0.005,1/12,5)
        self.edges = []
        for i in range(len(path)-1):
            self.edges.append(Edge(path[i],path[i+1], 1/50))
        self.best = self.get_path_cost(self.edges)

        self.path = []
        for i in range(len(self.edges)):
            if i == 0:
                self.path.append(self.edges[i].stnode)
            self.path.append(self.edges[i].ennode)




    def search(self):
        for i in range(1):

            # For each edge
            for i in range(0,int(len(self.edges)/1-5),8):

                b = np.random.randint(2,7)
                # i = np.random.randint(max(len(self.edges)-b,1))
                # Try set of velocities
                for vel in self.vels:
                    edges = list(self.edges)
                    for k in range(i,i+b):
                        if k <len(self.edges):
                            edges[k] = Edge(edges[k].stnode, edges[k].ennode,vel)
                    c = self.get_path_cost(edges)
                    if c < self.best:
                        self.best = c
                        self.pre = list(edges)
                # self.draw()

                if self.best != self.get_path_cost(self.edges):
                    self.edges = list(self.pre)
                    # self.draw()


        self.path = []
        for i in range(len(self.edges)):
            if i == 0 :
                self.path.append(self.edges[i].stnode)
            self.path.append(self.edges[i].ennode)


    # def draw(self):
    #     vs.pygame.event.get()

        # vs.draw()
        # for edge in self.edges:
        #     c = int(edge.vel*255*12)
            # pygame.draw.line(vs.screen, (c,c,c), vs.cartesian_to_screen(edge.st),
            #                  vs.cartesian_to_screen(edge.en), 5)
        # vs.pygame.display.flip()

    # Returns cost of complete path
    def get_path_cost(self, edges):
        cost = 0
        t_path = env.tstep
        l_path = 0
        # For each edge
        for edge in edges:
            # Add cost of the edge
            cost += self.get_edge_cost(edge, t_path)
            # Increase the time for next evaluation (velocity = 1/50, therefore time is 50 times space)
            t_path += edge.len/edge.vel
            l_path += edge.len


        # return cost / (t_path / 50) + t_path / 50
        return cost/max(l_path,0.0000001)/100

    def get_edge_cost(self, edge, t_i):
        vec = edge.en - edge.st
        length = np.linalg.norm(vec)

        # Divide edge into shorter edges to increase evaluation resolution
        reps = int(length / 0.2) + 2
        mults = np.linspace(0, 1, reps)
        cost = 0
        t_path = 0
        for j in range(reps - 1):
            current = edge.st + mults[j] * vec
            next = edge.st + mults[j + 1] * vec

            cost += cm.get_neighbour_cost(current, next, int(t_i + t_path)) * np.linalg.norm(current - next)

            # Increase the time for next evaluation (velocity = 1/50, therefore time is 50 times space)
            t_path += np.linalg.norm(next - current) / edge.vel

        return cost

    def get_path(self):
        path = []
        t = self.t_ini

        node = self.edges[0].stnode
        node.t = t
        path.append(self.edges[0].stnode)

        for edge in self.edges:
            t+= edge.len/edge.vel
            node = edge.ennode
            node.t = t
            path.append(node)
        return path

