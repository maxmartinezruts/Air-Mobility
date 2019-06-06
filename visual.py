import pygame
import numpy as np
import costmap as cm
from environment import env



render = 0
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

# Drawing Board
def draw():
    global render
    render +=1
    pygame.event.get()
    screen.fill((0, 0, 0))
    # screen.blit(image, (0, 0))
    if render % 4 ==0:
        for i in range(len(cm.xs)):
            for j in range(len(cm.ys)):
                w =  cm.get_dynamic_w(np.array([cm.xs[i], cm.ys[j]]),env.tstep)
                b = min(255, int(w * 100))
                pygame.draw.circle(screen, (b, 0, 0), cartesian_to_screen(np.array([cm.xs[i], cm.ys[j]])), 3)

    for drone in env.drones:
        pygame.draw.circle(screen, green, cartesian_to_screen(drone.pos),  5)

    for kitchen in env.kitchens:
        pygame.draw.circle(screen, yellow, cartesian_to_screen(kitchen.pos),  10)

    for order in env.orders:
        pygame.draw.circle(screen, white, cartesian_to_screen(order.pos),  5)

    pygame.display.flip()