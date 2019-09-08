#-------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Main (generates environment, calls the pathfinding search and runs the visualization simulation)
#-------------------------------#

# Import other scripts
import numpy as np
from environment import env
from drone import Drone
from order import Order
from kitchen import Kitchen
import visual as vs

# Create list where positions and directions are going to be recorded
for t in range(100000):
    env.pos_drones.append([])
    env.dir_drones.append([])

# Create environment (drones, and kitchens)
n_drones = 20
n_kitchens = 3
for i in range(n_kitchens):
    env.kitchens.append(Kitchen(i))
for i in range(n_drones):
    env.drones.append(Drone())

vs.draw()

# Determine a path for all drones
for drone in env.drones:
    drone.set_path()
    vs.draw()
    print(env.drones.index(drone))
vs.initialize_3d()

# Start and continue simulation
while True:
    print(env.tstep)
    # env.tstep +=1
    # Each 700 time steps create an order
    if env.tstep%7000 == 0:

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

            if len(available_drones) > 2:
                np.random.choice(available_drones).take_order(order)
                order.state = 'moving'

    # Execute drones actions
    for drone in env.drones:
        drone.act()

    # Draw scene
    vs.show_result()


