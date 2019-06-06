import numpy as np
np.random.seed(5)

from environment import env
from drone import Drone
from order import Order
from kitchen import Kitchen
import visual as vs


for t in range(100000):
    env.pos_drones.append([])

n_drones =  15
n_kitchens = 3

# Create environment (drones, and kitchens)
for i in range(n_kitchens):
    env.kitchens.append(Kitchen())

for i in range(n_drones):
    env.drones.append(Drone())

for drone in env.drones:
    drone.set_path()

# Start and continue simulation
while True:
    env.tstep +=1
    # Each 700 time steps create an order
    if env.tstep%700 == 0:

        # New order from random kitchen (could be rejected if range is too large)
        order = Order()
        env.orders.append(order)

    # Check for drones to take orders
    for order in env.orders:
        if order.state == 'waiting':

            # Check available drones in order's kitchen
            available_drones = []
            for drone in env.drones:
                if drone.is_available() and drone.kitchen == order.kitchen:
                    available_drones.append(drone)

            if len(available_drones) > 0:
                np.random.choice(available_drones).take_order(order)
                order.state = 'moving'

    # Execute drones actions
    for drone in env.drones:
        drone.act()

    # Draw scene
    vs.draw()


