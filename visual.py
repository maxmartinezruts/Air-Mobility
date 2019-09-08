# -------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: Includes all helpers to visualize the simulation in pygame (2D)
# -------------------------------#

import pygame
import numpy as np
import costmap as cm
from environment import env
from sklearn.cluster import KMeans
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import helpers as hp
import math

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

def polygon(poly):
    glBegin(GL_LINES)
    for i in range(len(poly)-1):

        glVertex3fv((poly[i][0],0,poly[i][1]))
        glVertex3fv((poly[i+1][0],0,poly[i+1][1]))
    glEnd()


def points(pts):
    quad = gluNewQuadric();
    for pt in pts:
        glColor3f(1.-pt[2]/20, pt[2]/20, 0.)

        glTranslatef(pt[0], pt[2]/4, pt[1]);
        gluSphere(quad, .1, 20, 20);
        glTranslatef(-pt[0], -pt[2]/4, -pt[1]);

        glBegin(GL_LINES)
        glVertex3fv((pt[0], pt[2]/4, pt[1]))
        glVertex3fv((pt[0], 0, pt[1]))
        glEnd()
        drawText((pt[0], pt[2]/4, pt[1]), str(pt[2])[:4])
        print(pt[2], str(pt[2])[:4])


def flip():
    pygame.display.flip()

def draw():


    global render
    render +=1
    pygame.event.get()
    screen.fill((0, 0, 0))
    image = pygame.image.load('Model/costmap.jpg')
    screen.blit(image, (0, 0))
    if render % 1 ==0:
        for i in range(len(cm.xs)):
            for j in range(len(cm.ys)):
                w =  cm.weights[i,j]
                b = min(255, int(w * 100)-100)
                # free = True
                # for drone in env.drones:
                #     if np.linalg.norm(np.array([cm.xs[i], cm.ys[j]])-drone.pos)<0.8:
                #         free = False
                # if free:
                #     pygame.draw.circle(screen, (200, 200, 200), cartesian_to_screen(np.array([cm.xs[i], cm.ys[j]])), 5)

    for poly in cm.polygons:
        if len(poly)>0:
            pygame.draw.polygon(screen, (150,150,150), [cartesian_to_screen([pol[0],pol[1]]) for pol in poly],3)

    for drone in env.drones:
        print(drone.pos)
        g = max(min(255,int(drone.pos[2]*12.5)),0)
        pygame.draw.circle(screen, (255-g,g,0), cartesian_to_screen(drone.pos),  5)

    for drone in env.pos_drones[env.tstep]:

        pygame.draw.circle(screen, green, cartesian_to_screen(drone),  5)


    for kitchen in env.kitchens:
        pygame.draw.circle(screen, yellow, cartesian_to_screen(kitchen.pos),  int(width/16.8/2*0.3))

    for order in env.orders:
        pygame.draw.circle(screen, white, cartesian_to_screen(order.pos),  5)
    if len(env.pos_drones[env.tstep])>7:
        X = []
        for drone in env.pos_drones[env.tstep]:

            X.append(np.array([drone[0],drone[1]]))
        kmeans = KMeans( random_state=0).fit(np.array(X))
        # for c in kmeans.cluster_centers_:
        #     arc(screen, white, Rect, start_angle, stop_angle, width=1) -> Rect

        for i in range(len(env.pos_drones[env.tstep])):
            if kmeans.labels_[i]%5==0:cl = green
            if kmeans.labels_[i]%5==1:cl = blue
            if kmeans.labels_[i]%5==2: cl = red
            if kmeans.labels_[i]%5==3: cl = yellow
            if kmeans.labels_[i]%5==4: cl = white

            # pygame.draw.circle(screen, cl, cartesian_to_screen(env.pos_drones[env.tstep][i]), 5)

    pygame.display.flip()


def initialize_3d():
    global angle
    pygame.init()

    dim = 8
    asp = 1
    display = (800, 800)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-dim * asp, +dim * asp, -dim * asp, +dim * asp, -dim * 10, +dim * 10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(45, 1, 0, 0)
    angle = 0

def drawText(position, textString):
    font = pygame.font.Font (None, 18)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


# Drawing Board
def show_result():
    global angle
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                glRotatef(3, 0, 1, 0)
                angle+=3*2*math.pi/360
            if event.key == pygame.K_RIGHT:
                glRotatef(3, 0, -1, 0)
                angle -=3*2*math.pi/360
            if event.key == pygame.K_DOWN:
                glRotatef(3, math.cos(angle), 0, math.sin(angle))
            if event.key == pygame.K_UP:
                glRotatef(3, -math.cos(angle), 0,- math.sin(angle))
            if event.key == pygame.K_1:
                env.tstep -= 5
            if event.key == pygame.K_2:
                env.tstep+=5

    glRotatef(0.2, 0, 1, 0)
    angle+=0.2*2*math.pi/360
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    pts = []
    glColor3f(1., 1., 1.)
    glLineWidth(1.)

    for i in range(len(env.pos_drones[env.tstep])):
        pts.append(env.pos_drones[env.tstep][i])

        # for j in range(5):
        #     glVertex3fv((env.pos_drones[env.tstep+j][i][0], env.pos_drones[env.tstep+j][i][2] / 4, env.pos_drones[env.tstep+j][i][1]))
        #     glVertex3fv((env.pos_drones[env.tstep+j+1][i][0], env.pos_drones[env.tstep+j+1][i][2] / 4, env.pos_drones[env.tstep+j+1][i][1]))
        # print(drone[2])
    #
    # for drone in env.drones:
    #     pts.append(drone.pos)

    points(pts)
    quad = gluNewQuadric();
    glColor3f(1, 1, 0)

    for kitchen in env.kitchens:

            glTranslatef(kitchen.pos[0], kitchen.pos[2] / 4, kitchen.pos[1]);
            gluSphere(quad, .15, 20, 20);
            glTranslatef(-kitchen.pos[0], -kitchen.pos[2] / 4, -kitchen.pos[1]);
    glColor3f(1., 1., 1.)
    for poly in cm.polygons:
        polygon(poly)

    glColor3f(0., 1., 0.)
    glLineWidth(1.)
    for i in range(len(env.pos_drones[env.tstep])):
        for j in range(len(env.pos_drones[env.tstep])):
            agl = hp.angle(env.dir_drones[env.tstep][i][:2],env.dir_drones[env.tstep][j][:2])
            angle_vs = hp.angle(env.dir_drones[env.tstep][i][:2], env.pos_drones[env.tstep][j][:2] - env.pos_drones[env.tstep][i][:2])

            radius = min((3*agl)**2/2,7)
            # radius = 2
            if radius > abs(env.pos_drones[env.tstep][i][2]-env.pos_drones[env.tstep][j][2]) and radius/13 > np.linalg.norm(env.pos_drones[env.tstep][i][:2]-env.pos_drones[env.tstep][j][:2])  and angle_vs <math.pi/4:
                if env.tstep>30 and agl > 1:
                    glColor3f(1., 0., 0.)
                    glLineWidth(4.)
                    glBegin(GL_LINES)
                    glVertex3fv((env.pos_drones[env.tstep][i][0], env.pos_drones[env.tstep][i][2]/4, env.pos_drones[env.tstep][i][1]))
                    glVertex3fv((env.pos_drones[env.tstep][j][0], env.pos_drones[env.tstep][j][2]/4, env.pos_drones[env.tstep][j][1]))
                    glEnd()
    # for path in env.paths:
    #     glBegin(GL_LINES)
    #
    #     for edge in path:
    #         glVertex3fv((edge.st[0], edge.st[2]/4, edge.st[1]))
    #         glVertex3fv((edge.en[0], edge.en[2]/4, edge.en[1]))
    #     glEnd()

    flip()
angle = 0
#

# gluPerspective(90, 800 / 600, .1, 1000)

# screen.fill((0, 0, 0))
# for i in range(env.n):
#     for j in range(env.n):
#         w = cm.weights[i,j]
#         b = min(255, int(w**3))*0.25
#         pygame.draw.circle(screen, (b, 0, 0), cartesian_to_screen(np.array([cm.xs[i], cm.ys[j]])), 5)
# pygame.image.save(screen, "Model/costmap.jpg")