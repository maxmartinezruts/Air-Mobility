import numpy as np
import random
import math
import pygame
import time
# import osmnx as ox
# G = ox.graph_from_place('Manhattan Island, New York City, New York, USA', network_type='drive')
# ox.plot_graph(G)
render = 0

pos_drones = []
for t in range(100000):
    pos_drones.append([])
g_t = 0

# Returns dot product of two vectors
def dotproduct(v1, v2):
    return sum((a * b) for a, b in zip(v1, v2))

# Returns lenght of a vector
def length(v):
    return math.sqrt(dotproduct(v, v))

# Returns angle between two vectors
def angle(v1, v2):
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

# Returns coordinates given a cartesian position
def get_coor_by_pos(pos):
    x = int(round((pos[0] - (-8)) / (8 - (-8)) * ((n - 1))))
    y = int(round((pos[1] - (-8)) / (8 - (-8)) * ((n - 1))))
    if x < 0: x = 0
    if x >= n: x = n - 1
    if y < 0: y = 0
    if y >= n: y = n - 1
    coor = np.array([x, y], dtype=int)
    return coor


# Convert coordinates form cartesian to screen coordinates (used to draw in pygame screen)
def cartesian_to_screen(car_pos):
    factor = 0.021
    screen_pos = np.array([center[0] * factor + car_pos[0], center[1] * factor - car_pos[1]]) / factor
    screen_pos = screen_pos.astype(int)
    return screen_pos

# Convert coordinates form screen to cartesian  (used to draw in pygame screen)
def screen_to_cartesian(screen_pos):
    factor = 0.021
    car_pos = np.array([screen_pos[0] - center[0], center[1] - screen_pos[1]]) * factor
    car_pos = car_pos.astype(float)
    return car_pos

# Get weight cost of a point p
def get_w(p):
    # P contains 3 dimensions: x, y and t
    sigma = 0.3
    w = 0

    # Gaussian function to apply weight
    for bump in bumps:
        w += 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (np.linalg.norm(bump - p[:2])) ** 2 / (2 * sigma ** 2)) * 0
    w += 0.1
    return w

# Get weight cost of a point p
def get_dynamic_w(p,t):
    w = 0
    sigma = 0.3

    # P contains 3 dimensions: x, y and t
    for pos in pos_drones[t]:
        if np.linalg.norm(p-pos)<0.8:
            w += 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (np.linalg.norm(pos - p)) ** 2 / (0.6 * sigma ** 2)) * 500

    return w


# Class point
class Point:
    def __init__(self, pos):
        self.pos = pos

# Class graph
class Graph:
    def __init__(self, p):
        self.path = p

    # Search
    def search(self):
        self.de_relax()
        self.try_new_pos()
        self.try_remove_worst()

    def try_new_pos(self):
        # Choose random node
        cps = []
        pps = []
        t_path = g_t
        for choice in range(1, len(self.path) - 1):
            cs = []
            ps = []
            len_edge =np.linalg.norm(self.path[choice+1].pos - self.path[choice].pos)

            for k in range(int(len_edge)*10):
                delta = np.random.randn(2)/len(self.path)
                p = self.path[choice].pos + (self.path[choice+1].pos - self.path[choice].pos)/len_edge*np.random.uniform(0,len_edge) + delta
                pygame.draw.circle(screen, white, cartesian_to_screen(p), 3)

                # specify which route to take into account
                if True:
                    c = self.get_edge_cost(self.path[choice - 1].pos, p, t_path) + self.get_edge_cost(p,
                                                                                              self.path[choice + 1].pos, t_path + np.linalg.norm(self.path[choice - 1].pos- p)*50)
                    cs.append(c)
                    ps.append(p)
            if len(cs) > 0:
                pt = ps[cs.index(min(cs))]
                path = list(self.path)
                path[choice] = Point(pt)
                cps.append(self.get_path_cost(path))
                pps.append(path)

            t_path += np.linalg.norm(self.path[choice - 1].pos-self.path[choice].pos)*50   # Review, not sure if right
        if len(cps) > 0:
            self.path = pps[cps.index(min(cps))]
            self.reconstruct_path(np.array([0, 1, 0], dtype=int))

    def de_relax(self):
        cps = []
        pps = []
        t_path = g_t
        for choice in range(0, len(self.path) - 1):
            # choice = random.randint(0,len(self.path)-2)
            cs = []
            ps = []
            len_edge =np.linalg.norm(self.path[choice+1].pos - self.path[choice].pos)
            for k in range(int(len_edge)*10):
                delta = np.random.randn(2)/len(self.path)
                p = self.path[choice].pos + (self.path[choice+1].pos - self.path[choice].pos)/len_edge*np.random.uniform(0,len_edge) + delta

                if True:
                    c = self.get_edge_cost(self.path[choice].pos, p, t_path) + self.get_edge_cost(p, self.path[choice + 1].pos, t_path + np.linalg.norm(self.path[choice].pos- p)*50)
                    cs.append(c)
                    ps.append(p)
            cl = min(int(t_path), 255)
            c2 = min(int(t_path+len_edge*50), 255)

            pygame.draw.circle(screen, (cl, cl, cl), cartesian_to_screen(self.path[choice].pos), 10)
            pygame.draw.circle(screen, (c2, c2, c2), cartesian_to_screen(self.path[choice+1].pos), 10)

            pygame.display.flip()
            # time.sleep(0.01)
            if len(cs) > 0:
                path = list(self.path)
                path.insert(choice + 1, Point(ps[cs.index(min(cs))]))
                cps.append(self.get_path_cost(path))
                pps.append(path)
            t_path += len_edge*50  # Review, not sure if right

        # pygame.display.flip()
        # time.sleep(1)
        if len(cps) > 0:
            self.path = pps[cps.index(min(cps))]
            self.reconstruct_path(np.array([1, 0, 0], dtype=int))

    def try_remove_worst(self):
        cs = []
        for p in range(1, len(self.path) - 1):
            path = list(self.path)
            path.pop(p)
            cost = self.get_path_cost(path)
            cs.append(cost)
        if len(cs) > 0:
            if min(cs) < self.get_path_cost(self.path):
                choice = cs.index(min(cs)) + 1
                self.path.pop(choice)
                print(min(cs), self.get_path_cost(self.path))
                self.reconstruct_path(np.array([1, 1, 1], dtype=int))

    def get_path_cost(self, path):
        cost = 0
        t_path = g_t
        for p in range(0, len(path) - 1):

            cost += self.get_edge_cost(path[p].pos, path[p + 1].pos, t_path)
            t_path += np.linalg.norm(path[p].pos- path[p + 1].pos)*50

        return cost

    def get_edge_cost(self, i, f, t_i):
        vec = f - i
        length = np.linalg.norm(vec)
        reps = int(length / 0.5) + 2
        mults = np.linspace(0, 1, reps)
        cost = 0
        for j in range(reps - 1):
            current = i + mults[j] * vec
            next = i + mults[j + 1] * vec
            wi = weights[get_coor_by_pos(current)[0], get_coor_by_pos(current)[1]] + get_dynamic_w(current,int(t_i))
            wf = weights[get_coor_by_pos(next)[0], get_coor_by_pos(next)[1]] + get_dynamic_w(next,int(t_i + length*50))
            cost += (wi + wf) / 2 * np.linalg.norm(current - next)

        return cost

    def reconstruct_path(self, color):
        pygame.event.get()

        draw()
        for p in range(len(self.path) - 1):
            c = color * 255
            pygame.draw.line(screen, c, cartesian_to_screen(self.path[p].pos),
                             cartesian_to_screen(self.path[p + 1].pos), 3)
            pygame.draw.circle(screen, white, cartesian_to_screen(self.path[p].pos), 3)

        print(self.get_path_cost(self.path), len(self.path))
        pygame.display.flip()
        return self.path

# Drawing Board
def draw():
    global render
    render +=1
    pygame.event.get()
    screen.fill((0, 0, 0))
    # screen.blit(image, (0, 0))
    print(pos_drones[0])
    print(pos_drones[1])
    if render % 4 ==0:
        for i in range(len(xs)):
            for j in range(len(ys)):
                w =  get_dynamic_w(np.array([xs[i], ys[j]]),g_t+100)
                b = min(255, int(w * 100))
                pygame.draw.circle(screen, (b, 0, 0), cartesian_to_screen(np.array([xs[i], ys[j]])), 3)

    for drone in drones:
        pygame.draw.circle(screen, green, cartesian_to_screen(drone.pos),  5)

    for kitchen in kitchens:
        pygame.draw.circle(screen, yellow, cartesian_to_screen(kitchen.pos),  10)

    for order in orders:
        pygame.draw.circle(screen, white, cartesian_to_screen(order.pos),  5)

    pygame.display.flip()

# Decide which kitchen to return to
def pick_kitchen(pos):

    # First select best based on which kitchen is closest
    min_dist = 1000000
    for k in kitchens:
        if np.linalg.norm(pos-k.pos) < min_dist:
            min_dist = np.linalg.norm(pos-k.pos)
            best = k

    # But overwrite if a kitchen has few available drones
    for k in kitchens:
        n=0
        # Count number of available drones
        for drone in drones:
            if drone.kitchen == k and drone.state =='available':
                n+=1

        # In case number of available drones is low in one kitchen, choose this one
        if n < 5:
            best = k
    return best

# Class Drone
class Drone:
    def __init__(self):
        global pos_drones       # Maps t to pos

        # Set of states
        # 1. Available  - if drone is waiting in kitchen
        # 2. Carrying   - if drone is flying carrying an order
        # 3. Free       - if frone is flying not carrying an order

        # Cartesian position of the drone
        self.pos = np.random.randn(2)*4

        # Start drone as free
        self.state = 'free'

        # Start drone with no orders
        self.order = None

        # Allocate random kitchen
        self.kitchen = random.choice(kitchens)

        # Number of iterations
        self.recursion_depth = 15

        path = [Point(self.pos), Point(self.kitchen.pos)]
        graph = Graph(path)
        for i in range(15):
            graph.search()

        self.path = graph.path[1:]

        copypath = list(graph.path[1:])
        copypos = np.array(list(self.pos))
        # Add drone location in time to cost map
        t = g_t
        while len(copypath) > 0:
            step = (copypath[0].pos - copypos) / (np.linalg.norm(copypath[0].pos - copypos) + 0.00001) / 50
            copypos += step
            print(copypos, t)
            pos_drones[t].append(np.array(list(copypos)))
            if np.linalg.norm(copypath[0].pos - copypos) < (4 / 50):
                copypath.pop(0)
            t+=1

    # Perform action
    def act(self):
        global orders

        # If carrying an order, move to order position, if arrive: drop and pick a kitchen to return to
        if self.state == 'carrying':
            # Advance to target
            self.navigate()
            if np.linalg.norm(self.order.pos - self.pos) < (4/50):
                self.state = 'free'
                self.order.state = 'delivered'
                self.kitchen = pick_kitchen(self.pos)
                path = [Point(self.pos), Point(self.kitchen.pos)]
                graph = Graph(path)
                for i in range(self.recursion_depth):
                    graph.search()

                self.path = graph.path[1:]
                orders.remove(self.order)

        # If flying but not carrying, move to kitchen, if arrive: change state to available
        if self.state == 'free':
            self.navigate()
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
        self.order = order
        self.state = 'carrying'
        path = [Point(self.pos), Point(order.pos)]
        graph = Graph(path)
        for i in range(self.recursion_depth):
            graph.search()

        self.path = graph.path[1:]
        print(self.path)
        print(self.path[::-1])

    def navigate(self):
        step = (self.path[0].pos - self.pos) / (np.linalg.norm(self.path[0].pos - self.pos)+0.00001) / 50
        self.pos += step
        if np.linalg.norm(self.path[0].pos - self.pos) < (4/50):
            self.path.pop(0)
            print('fdasasdfsdfadfsd')

class Order:
    def __init__(self):

        # Set of states
        # 1. waiting    - if order has not yet been taken by a drone
        # 2. moving     - if drone already took the order

        self.pos = np.random.randn(2)*4
        self.start_time = time.time()
        self.state = 'waiting'
        self.kitchen = random.choice(kitchens)


class Kitchen:
    def __init__(self):
        self.pos = np.random.randn(2)*2.4

# Screen parameters
width = 800
height = 800
center = np.array([width/2, height/2])
screen = pygame.display.set_mode((width, height))

# Colors
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
white = (255, 255, 255)
yellow = (255,255, 0)



fpsClock = pygame.time.Clock()

fps = 400
n = 70
xs = np.linspace(-8, 8, n)
ys = np.linspace(-8, 8, n)
np.random.seed(1)
bumps = []
for i in np.linspace(-6,6,9):
    for j in np.linspace(-6,6,9):
        bumps.append([i,j])
bumps = np.array(bumps)
bumps = np.random.uniform(-6,6,(50,2))
weights = np.zeros((n, n))

for i in range(len(xs)):
    for j in range(len(ys)):
        w = get_w(np.array([xs[i], ys[j]]))
        weights[i, j] = w
        b = min(255, int(w * 100))
        pygame.draw.circle(screen, (b, 0, 0), cartesian_to_screen(np.array([xs[i], ys[j]])), 3)

pygame.image.save(screen, "geek.jpg")
screen.fill((0, 0, 0))
image = pygame.image.load('geek.jpg')
# print(image)
screen.blit(image, (0, 0))
pygame.display.flip()

waiting = True
graphs = []
path = []


# Define list of robots

n_drones = 15
n_kitchens = 3

drones = []
kitchens = []
orders = []

for i in range(n_kitchens):
    kitchens.append(Kitchen())

for i in range(n_drones):
    drones.append(Drone())

t = 0
# Start and continue simulation
while True:
    t+=1
    g_t +=1

    # Each 70 time steps create an order
    if t%70 == 0:

        # New order from random kitchen (could be rejected if range is too large)
        order = Order()
        orders.append(order)

    # Check for drones to take orders
    for order in orders:
        if order.state == 'waiting':

            # Check available drones in order's kitchen
            available_drones = []
            for drone in drones:
                if drone.is_available() and drone.kitchen == order.kitchen:
                    available_drones.append(drone)

            if len(available_drones) > 0:
                random.choice(available_drones).take_order(order)
                order.state = 'moving'

    for drone in drones:
        drone.act()
    draw()


