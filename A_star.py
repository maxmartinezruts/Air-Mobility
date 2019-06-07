import numpy as np
import math
import random
import pygame
from environment import env
from costmap import weights
import pygame
from visual import screen, cartesian_to_screen, green, yellow
from shapely.geometry import Polygon, MultiPolygon, mapping, Point, MultiPoint
# import osmnx as ox

xValues = np.linspace(-8, 8, env.n)
yValues = np.linspace(-8, 8, env.n)

def heuristic_cost_estimate(st, en):
    return  np.linalg.norm(st.pos - en.pos)
            #+ abs(math.atan2(math.sin(st.ang-en.ang), math.cos(st.ang-en.ang)))/5

def heuristic_cost_estimate_neighbour(st, en):
    w = ((weights[st.coor[0],st.coor[1]]-0.1)**10+(weights[en.coor[0],en.coor[1]]-0.1)**10)/2
    return  np.linalg.norm(st.pos - en.pos)*w


def get_pos_by_coor(coor):
    return np.array([xValues[coor[0]], yValues[coor[1]]])

def get_coor_by_pos(pos):
    x =int(round((pos[0]-(-8))/(8-(-8))*((env.n-1))))
    y =int(round((pos[1]-(-8))/(8-(-8))*((env.n-1))))
    # k =int(round((pos[2]-(0))/(2*math.pi-(0))*((m-1))))
    coor = np.array([x,y], dtype=int)
    return coor

class Node:
    def __init__(self, state, parent, graph):
        self.gScore = math.inf
        self.fScore = math.inf
        self.parent = parent
        self.state = state
        self.pos =state
        self.coor = get_coor_by_pos(np.array([self.pos[0],self.pos[1]]))
        self.graph = graph
        self.graph.availableCoors[self.coor[0]][self.coor[1]] = self

    def get_neighbors(self):


        neightbours = []
        current_coor = get_coor_by_pos(self.pos)
        for i in range(-2,3):
            for j in range(-2,3):
                if not( i==0 and j==0) and 0<= current_coor[0]+i < env.n and 0 <= current_coor[1]+j < env.n:
                    neighbourCoor = current_coor + np.array([i,j])
                    neighbourPos = get_pos_by_coor(neighbourCoor)
                if (self.graph.availableCoors[neighbourCoor[0]][neighbourCoor[1]] == 0):
                    node = Node(neighbourPos , self, self.graph)
                else:
                    node = self.graph.availableCoors[neighbourCoor[0]][neighbourCoor[1]]
                neightbours.append(node)
        return  neightbours


class Graph:
    def __init__(self):
        self.all = []
        self.availableCoors = [[0 for j in range(env.n)] for i in range(env.n)]

    def prepare(self, start, end):
        print('new')
        self.start = start
        self.start.gScore = 0
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
            self.reconstruct_path(self.start, current,green,1)
            #print(current.pos)
            if np.linalg.norm(current.pos - self.end.pos) <0.3:
                    #and current.ang == self.end.ang:
                print(current.parent)

                return self.reconstruct_path(self.start, current,yellow,3)

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
                neighbor.fScore = neighbor.gScore  + heuristic_cost_estimate(neighbor, self.end)

    def add_node(self, state):
        node = Node(state, self)
        self.availableCoors[coor[0]][coor[1]] = node
        self.all.append(node)

        return node
    def reconstruct_path(self, start, end, color, w):
        pygame.event.get()

        path = []
        current = end
        while current.parent != None:

            current = current.parent
            path.append(current)

        for p in range(len(path)-1):
            pygame.draw.line(screen, color, cartesian_to_screen(path[p].pos),cartesian_to_screen(path[p+1].pos), w)
        pygame.display.flip()
        self.path = path[::-1]
        return path
