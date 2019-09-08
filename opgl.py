# -------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Use of OpenGL to visualize all drone paths in a 3d environment
# -------------------------------#


import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
from shapely.geometry import Polygon, MultiPolygon, mapping, Point, MultiPoint
import json

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


with open('Model/data_city_100.txt') as json_file:
    polygons = json.load(json_file)
polygons_obj = [Polygon([(p[0], p[1]) for p in polygon]) for polygon in polygons]


# Convert coordinates form cartesian to screen coordinates (used to draw in pygame screen)
def cartesian_to_screen(car_pos):
    factor = 0.02
    screen_pos = np.array([center[0] * factor + car_pos[0], center[1] * factor - car_pos[1]]) / factor
    screen_pos = screen_pos.astype(int)
    return screen_pos

# Convert coordinates form screen to cartesian  (used to draw in pygame screen)
def screen_to_cartesian(screen_pos):
    factor = 0.02
    car_pos = np.array([screen_pos[0] - center[0], center[1] - screen_pos[1]]) * factor
    car_pos = car_pos.astype(float)
    return car_pos

vertices= (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
    )

vertices2= (
    (1, -1, -2),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )
def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def Cube2():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices2[vertex])
    glEnd()

def polygon(poly):
    glBegin(GL_LINES)
    for i in range(len(poly)-1):

        glVertex3fv((poly[i][0],0,poly[i][1]))
        glVertex3fv((poly[i+1][0],0,poly[i+1][1]))
    glEnd()



def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)


    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(-60, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0,3, -13)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glRotatef(1, 0, 3, 0)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        for poly in polygons:
            polygon(poly)
        pygame.display.flip()
        pygame.time.wait(10)


main()