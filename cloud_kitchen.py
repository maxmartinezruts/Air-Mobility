import numpy as np
import random
import math
import pygame
import time



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


# Convert coordinates form cartesian to screen coordinates (used to draw in pygame screen)
def cartesian_to_screen(car_pos):
    factor = 1
    screen_pos = np.array([center[0]*factor+car_pos[0],center[1]*factor+car_pos[1]])/factor
    screen_pos = screen_pos.astype(int)
    return screen_pos

# Drawing Board
def draw():
    pygame.event.get()
    screen.fill((0, 0, 0))

    for drone in drones:
        pygame.draw.circle(screen, green, cartesian_to_screen(drone.pos),  5)

    for kitchen in kitchens:
        pygame.draw.circle(screen, red, cartesian_to_screen(kitchen.pos),  5)

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

        # Set of states
        # 1. Available  - if drone is waiting in kitchen
        # 2. Carrying   - if drone is flying carrying an order
        # 3. Free       - if frone is flying not carrying an order

        # Cartesian position of the drone
        self.pos = np.random.randn(2)*200

        # Start drone as free
        self.state = 'free'

        # Start drone with no orders
        self.order = None

        # Allocate random kitchen
        self.kitchen = random.choice(kitchens)



    # Perform action
    def act(self):
        global orders

        # If carrying an order, move to order position, if arrive: drop and pick a kitchen to return to
        if self.state == 'carrying':
            # Advance to target
            step = (self.order.pos - self.pos )/ np.linalg.norm(self.order.pos - self.pos)
            self.pos += step
            if np.linalg.norm(self.order.pos - self.pos) < 4:
                self.state = 'free'
                self.order.state = 'delivered'
                self.kitchen = pick_kitchen(self.pos)
                orders.remove(self.order)

        # If flying but not carrying, move to kitchen, if arrive: change state to available
        if self.state == 'free':
            # Advance to kitchen
            step = (self.kitchen.pos - self.pos) / np.linalg.norm(self.kitchen.pos - self.pos)
            self.pos += step
            if np.linalg.norm(self.kitchen.pos - self.pos) < 4:
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

class Order:
    def __init__(self):

        # Set of states
        # 1. waiting    - if order has not yet been taken by a drone
        # 2. moving     - if drone already took the order

        self.pos = np.random.randn(2)*200
        self.start_time = time.time()
        self.state = 'waiting'
        self.kitchen = random.choice(kitchens)


class Kitchen:
    def __init__(self):
        self.pos = np.random.randn(2)*130

# Define list of robots

n_drones = 30
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


