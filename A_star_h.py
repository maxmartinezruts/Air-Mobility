#-------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: A* Algorithm used to find the most-costeffective 'height' path once the 2D path has been found and optimized
#-------------------------------#

import numpy as np
import math
from environment import env
import pygame
from visual import cartesian_to_screen, draw, flip, screen
import costmap as cm


# Model new cost map (abstract, same for velocity actually)
zValues = np.linspace( 0, 20, env.m)


def heuristic_cost_estimate(st, en):
    return  np.linalg.norm(st.pos - en.pos)*1

def heuristic_cost_estimate_neighbour(st, en):
    w = cm.get_separation_cost(st.pos,en.pos,(int((st.t+en.t)/2)))
    return  np.linalg.norm(st.pos[:2] - en.pos[:2])*w


def get_h_by_coor(coor):
    return zValues[coor]

def get_coor(h):
    z =int(round(h/20*((env.m-1))))
    return z

class Node:
    def __init__(self, parent, graph, step,h, end=False):
        self.gScore = math.inf
        self.fScore = math.inf
        self.parent = parent
        self.coor = get_coor(h)
        self.step = step
        self.pos = np.array([graph.path_2d[step].pos[0],graph.path_2d[step].pos[1],h])
        self.t = graph.path_2d[step].t
        self.graph = graph
        if not end:
            self.graph.availableCoors[step][self.coor] = self

    def get_neighbors(self):

        neightbours = []

        for k in range(-2, 3):
            if  0 <= self.coor+k < env.m and 0<=self.step<len(self.graph.path_2d)-1:
                neighbourCoor = np.array([self.step + 1, self.coor + k])
                if (self.graph.availableCoors[neighbourCoor[0]][neighbourCoor[1]] == 0):
                    node = Node(self, self.graph, self.step+1, get_h_by_coor(neighbourCoor[1]))
                    neightbours.append(node)

                else:
                    node = self.graph.availableCoors[neighbourCoor[0]][neighbourCoor[1]]
        return  neightbours


class Graph:
    def __init__(self, path):
        self.all = []
        self.path_2d = path
        print('lenpath',len(path))
        self.availableCoors = [[0 for j in range(env.m)] for i in range(len(self.path_2d))]

    def prepare(self, start, end):
        print('new')
        self.start = start
        self.start.gScore = 0
        self.start.t = 0.0

        self.end = end
        # The set of nodes already evaluated
        self.closedSet = []

        # The set of currently discovered nodes that are not evaluated yet.
        #  Initially, only the start node is known.

        self.openSet = [start]

        #         # For each node, which node it can most efficiently be reached from.
        #         # If a node can be reached from many nodes, cameFrom will eventually contain the
        #         # most efficient previous step.

    def search(self):
        current = self.start
        while len(self.openSet) > 0:  # While not close enough to end
            minScore = math.inf
            for node in self.openSet:
                if node.fScore < minScore:
                    minScore = node.fScore
                    current = node

            # current = the node in openSet having the lowest fScore[] value
            self.reconstruct_path(current,(0,255,0),3)
            print(current.step, len(self.path_2d)-1, current.coor)
            if current.step == len(self.path_2d)-1  and current.coor == 0:
                print(current.fScore)
                return self.reconstruct_path(current,(255,255,255),3)


            self.openSet.remove(current)
            self.closedSet.append(current)
            neighbors = current.get_neighbors()
            for neighbor in neighbors:
                if neighbor in self.closedSet:
                    continue  # Ignore the neighbor which is already evaluated.

                # The distance from start to a neighbor
                tentative_gScore = current.gScore + heuristic_cost_estimate_neighbour(current, neighbor)

                if neighbor not in self.openSet:  # Discover a new node
                    self.openSet.append(neighbor)
                elif tentative_gScore >= neighbor.gScore:
                    continue
                # This path is the best until now. Record it!
                neighbor.parent = current
                neighbor.gScore = tentative_gScore
                hce = heuristic_cost_estimate(neighbor, self.end)
                neighbor.fScore = (tentative_gScore+ hce)
                print(neighbor.fScore)

    def reconstruct_path(self, end, color, w):
        pygame.event.get()

        path = []
        current = end
        while current != None:
            path.append(current)
            current = current.parent

        # edges = []
        # for i in range(len(path) - 1):
        #     edges.append([(path[i].pos[0], path[i].pos[2] / 50, path[i].pos[1]),(path[i + 1].pos[0], path[i + 1].pos[2] / 50, path[i + 1].pos[1])])
        #     # edges.append([(path[i].pos[0], 0, path[i].pos[1]),(path[i + 1].pos[0], 0, path[i + 1].pos[1])])
        #
        # draw(edges,False)




        for p in range(len(path)-1):
            g = max(min(255,int(path[p+1].coor + path[p].coor)/2*12.75),0)
            pygame.draw.line(screen, (0,g,0), cartesian_to_screen(path[p].pos),cartesian_to_screen(path[p+1].pos), w)
        pygame.display.flip()
        if w ==3:
            print('finished')
        self.path = path[::-1]

        return path
