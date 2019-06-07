import pygame
from point import Point
import numpy as np
import costmap as cm
import visual as vs
from environment import env
import helpers as hp



# Class graph
class Graph:
    def __init__(self, p):

        # Initialize path
        self.path = p

    # Search
    def search(self):
        self.creation()
        self.shifting()
        self.deletion()

    def shifting(self):
        cps = []
        pps = []

        # For each point in path (excluding start and end), try to shift it to several different positions and select the one leading to minimum cost
        for choice in range(1, len(self.path) - 1):
            cs = []
            ps = []
            len_edge = np.linalg.norm(self.path[choice+1].pos - self.path[choice].pos)

            # Create a number of poits located in a 2D gaussian distribution located at the edge examined
            for k in range(int(len_edge)*10):
                delta = np.random.randn(2)/len(self.path)*10
                p = self.path[choice].pos + (self.path[choice+1].pos - self.path[choice].pos)/len_edge*np.random.uniform(0,len_edge) + delta
                if not cm.intersects(p):
                    # Substitude examined point by generated point, evaluate the new cost of the path and save
                    path = list(self.path)
                    path[choice] = Point(p)
                    c = self.get_path_cost(path)
                    cs.append(c)
                    ps.append(p)

            # Select shift operation that leads to the lowest cost
            if len(cs) > 0:
                pt = ps[cs.index(min(cs))]
                path = list(self.path)
                path[choice] = Point(pt)
                cps.append(self.get_path_cost(path))
                pps.append(path)

        # Select point that lead to the lowest cost shift operation and shift it
        if len(cps) > 0:
            self.path = pps[cps.index(min(cps))]
            self.reconstruct_path(np.array([0, 1, 0], dtype=int))

    def creation(self):
        cps = []
        pps = []

        # For each edge in the path, try to create a new point in several different positions and select the one leading to minimum cost
        for choice in range(0, len(self.path) - 1):
            cs = []
            ps = []
            len_edge =np.linalg.norm(self.path[choice+1].pos - self.path[choice].pos)

            # Create a number of poits located in a 2D gaussian distribution located at the edge examined
            for k in range(int(len_edge)*10):
                delta = np.random.randn(2)/len(self.path)*10
                p = self.path[choice].pos + (self.path[choice+1].pos - self.path[choice].pos)/len_edge*np.random.uniform(0,len_edge) + delta

                if not cm.intersects(p):
                    # Create generated point, evaluate the new cost of the path and save
                    path = list(self.path)
                    path.insert(choice + 1, Point(p))
                    c = self.get_path_cost(path)
                    cs.append(c)
                    ps.append(p)

            # Select creation operation that leads to the lowest cost
            if len(cs) > 0:
                path = list(self.path)
                path.insert(choice + 1, Point(ps[cs.index(min(cs))]))
                cps.append(self.get_path_cost(path))
                pps.append(path)

        # Select point that lead to the lowest cost creation operation and create it
        if len(cps) > 0:
            self.path = pps[cps.index(min(cps))]
            self.reconstruct_path(np.array([1, 0, 0], dtype=int))


    def deletion(self):
        cs = []

        # Virtually remove each point in the graph and delete the one that will lead to the greatest reduction in cost
        for p in range(1, len(self.path) - 1):
            path = list(self.path)
            path.pop(p)
            cost = self.get_path_cost(path)
            cs.append(cost)

        # Select point that lead to the lowest cost deletion operation delete it
        if len(cs) > 0:
            if min(cs) < self.get_path_cost(self.path):
                choice = cs.index(min(cs)) + 1
                self.path.pop(choice)
                # print(min(cs), self.get_path_cost(self.path))
                self.reconstruct_path(np.array([1, 1, 1], dtype=int))

    # Returns cost of complete path
    def get_path_cost(self, path):
        cost = 0
        t_path = env.tstep

        # For each edge
        for p in range(0, len(path) - 1):
            # Add cost of the edge
            cost += self.get_edge_cost(path[p].pos, path[p + 1].pos, t_path)

            # Increase the time for next evaluation (velocity = 1/50, therefore time is 50 times space)
            t_path += np.linalg.norm(path[p].pos- path[p + 1].pos)*50

        return cost

    def get_edge_cost(self, i, f, t_i):

        vec = f - i
        length = np.linalg.norm(vec)

        # Divide edge into shorter edges to increase evaluation resolution
        reps = int(length / 0.5) + 2
        mults = np.linspace(0, 1, reps)
        cost = 0
        t_path = 0
        for j in range(reps - 1):
            current = i + mults[j] * vec
            next = i + mults[j + 1] * vec

            # Establish weights of the start and end points by evaluation spatial and time-dependent cost
            wi = cm.weights[hp.get_coor_by_pos(current)[0], hp.get_coor_by_pos(current)[1]] + cm.get_dynamic_w(current,int(t_i+t_path))
            wf = cm.weights[hp.get_coor_by_pos(next)[0], hp.get_coor_by_pos(next)[1]] + cm.get_dynamic_w(next,int(t_i + t_path + np.linalg.norm(current-next)*50))

            # Cost set to average of start and end weights * distance of edge
            cost += (wi + wf) / 2 * np.linalg.norm(current - next)

            # Increase the time for next evaluation (velocity = 1/50, therefore time is 50 times space)
            t_path += np.linalg.norm(next-current)*50

        return cost

    # Display path in screen
    def reconstruct_path(self, color):
        vs.pygame.event.get()
        vs.draw()
        for p in range(len(self.path) - 1):
            c = color * 255
            pygame.draw.line(vs.screen, c, vs.cartesian_to_screen(self.path[p].pos),
                             vs.cartesian_to_screen(self.path[p + 1].pos), 3)
            pygame.draw.circle(vs.screen, vs.white, vs.cartesian_to_screen(self.path[p].pos), 3)

        vs.pygame.display.flip()
        return self.path
