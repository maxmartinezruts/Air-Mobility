# -------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: NOT USED, attempt to create shortest path algorithm to find efficient velocities, resulted not to be efficient
# -------------------------------#

import numpy as np
import math
import random
import pygame
from environment import env
from costmap import weights
import pygame
from visual import screen, cartesian_to_screen, green, yellow
import costmap as cm
import time
from shapely.geometry import Polygon, MultiPolygon, mapping, Point, MultiPoint
# import osmnx as ox

xValues = np.linspace(-8, 8, env.n)
yValues = np.linspace(-8, 8, env.n)

def heuristic_cost_estimate_neighbour(st, en, t_pt):
    cost = cm.get_neighbour_cost(st, en, int(t_pt)) * np.linalg.norm(st - en)
    return  cost


class Node:
    def __init__(self, t, parent, idx, graph):
        self.gScore = math.inf
        self.parent = parent
        self.pos = graph.points[idx].pos
        # self.vel = np.linalg.norm(parent.pos-self.pos)/(t-parent.t)
        self.t = t
        self.idx = idx
        self.graph = graph
        self.dist = 0
        self.fScore = math.inf

    def get_neighbors(self):
        neightbours = self.graph.nodes_by_idx[self.idx+1]
        final = []
        for neightbour in neightbours:
            v = np.linalg.norm(neightbour.pos-self.pos)/(neightbour.t - self.t)
            if v < 0.11 and v > 0.002:
                final.append(neightbour)
            print(len(final))
        return  final

class Graph:

    def __init__(self, points, t_start):
        self.points = points



        print('new')
        self.start = Node(t_start,None,0,self)

        self.start.gScore = 0

        # The set of nodes already evaluated
        self.closedSet = []

        # The set of currently discovered nodes that are not evaluated yet.
        #  Initially, only the start node is known.

        self.openSet = [self.start]

        #         # For each node, which node it can most efficiently be reached from.
        #         # If a node can be reached from many nodes, cameFrom will eventually contain the
        #         # most efficient previous step.
        self.edges = []
        self.l_path = 0
        for i in range(len(points) - 1):
            points[i].dist = self.l_path
            e =Edge(points[i], points[i + 1])
            self.edges.append(e)
            self.l_path += e.len

        points[-1].dist = self.l_path


        self.path = []
        for i in range(len(self.edges)):
            if i == 0:
                self.path.append(self.edges[i].stnode)
            self.path.append(self.edges[i].ennode)
        self.nodes_by_idx = []
        self.l_path = 0

        for idx in range(len(points)):
            nodes = []
            for t in np.linspace( self.l_path/0.002,  self.l_path/0.1, 1 + 4 * idx):
                nodes.append(Node(t, None, idx, self))
            self.nodes_by_idx.append(nodes)
            self.l_path += self.edges[i].len
        # print(self.nodes_by_idx[3])
        for node in self.nodes_by_idx[3]:
            print(node.idx, node.t)

    def search(self):
        current = self.start

        while len(self.openSet) > 0:  # While not close enough to end
            # print('------------')
            # print(current.gScore)
            minScore = math.inf
            for node in self.openSet:
                if node.fScore < minScore:
                    minScore = node.fScore
                    current = node

            # current = the node in openSet having the lowest fScore[] value
            self.reconstruct_edges(self.start, current,green,1)
            if current.idx == len(self.points)-2:
                # print(len(self.points)-2)

                p = self.reconstruct_edges(self.start, current,yellow,3)

                return p

            self.openSet.remove(current)
            self.closedSet.append(current)

            neighbors = current.get_neighbors()
            for neighbor in neighbors:
                if neighbor in self.closedSet:
                    continue  # Ignore the neighbor which is already evaluated.

                # The distance from start to a neighbor
                tentative_gScore = current.gScore + heuristic_cost_estimate_neighbour(current.pos, neighbor.pos, (current.t))

                if neighbor not in self.openSet:  # Discover a new node
                    self.openSet.append(neighbor)
                elif tentative_gScore >= neighbor.gScore:
                    continue
                # This path is the best until now. Record it!
                neighbor.parent = current
                neighbor.gScore = tentative_gScore
                neighbor.fScore = neighbor.gScore + (self.l_path-neighbor.dist)
                # print(neighbor.fScore, neighbor.idx)


    def reconstruct_edges(self, start, end, color, w):
        pygame.event.get()

        edges = []
        current = end
        while current.parent != None:
            current = current.parent
            edge = Edge(self.points[current.idx],self.points[current.idx+1])
            edges.append(edge)

        for e in edges:
            c = min(int(e.vel * 20 * 255), 255)
            # if e.vel!=0.02:
                # print(e.vel)
            pygame.draw.line(screen, (c,c,c), cartesian_to_screen(e.st),cartesian_to_screen(e.en), 4)
        pygame.display.flip()
        self.edges = edges[::-1]

        return edges

class Edge:
    def __init__(self, st, en):
        self.st = st.pos
        self.en = en.pos
        self.vel = np.linalg.norm(en.pos-st.pos)/(en.t-st.t)
        self.len = np.linalg.norm(self.en-self.st)
        self.stnode = st
        self.ennode = en

