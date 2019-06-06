import numpy as np
from environment import env


# Get static cost given a point in space
def get_w(p):
    # P contains 3 dimensions: x, y and t
    sigma = 0.3

    # Initialize cost to 0
    w = 0

    # Gaussian function to apply cost near bumps
    for bump in bumps:
        w += 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (np.linalg.norm(bump - p[:2])) ** 2 / (2 * sigma ** 2)) * 0
    w +=0.1
    return w

# Get dynamic cost accouting with known objects in time
def get_dynamic_w(p,t):
    w = 0
    sigma = 0.3     # Strandard deviation

    # For each drone position at the given timestep, add gaussian function to apply cost near drones
    for pos in env.pos_drones[t]:
        w += 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (np.linalg.norm(pos - p)) ** 2 / (2 * sigma ** 2)) * 200

    # Add extra weight (otherwise it would not care about finding shorter paths, as all weights would be 0 in locations with no extra cost)
    w += 0.01
    return w

# Create spatial grid
xs = np.linspace(-8, 8, env.n)
ys = np.linspace(-8, 8, env.n)

# Use seed to generate same results for each simulation even using random events

# Generate bumps at random positions to illustrate the concept of cost map
bumps = []
for i in np.linspace(-6,6,9):
    for j in np.linspace(-6,6,9):
        bumps.append([i,j])
bumps = np.array(bumps)
bumps = np.random.uniform(-6,6,(50,2))


# Create cost grid (map) (only spatial, no time)
weights = np.zeros((env.n, env.n))
for i in range(len(xs)):
    for j in range(len(ys)):
        w = get_w(np.array([xs[i], ys[j]]))
        weights[i, j] = w
