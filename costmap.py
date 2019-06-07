import numpy as np
from environment import env
import osmnx as ox
import math
from shapely.geometry import Polygon, MultiPolygon, mapping, Point, MultiPoint
import matplotlib.pyplot as plt
import json
from descartes import PolygonPatch
import pygame

with open('data.txt') as json_file:
    polygons = json.load(json_file)

polygons_obj = [Polygon([(p[0], p[1]) for p in polygon]) for polygon in polygons]


def intersects(p):
    res = False
    for pol in polygons_obj:
        if pol.contains(Point(p[0],p[1])):
            res = True
    return res
# # Get static cost given a point in space
# def get_w(p):
#     # P contains 3 dimensions: x, y and t
#     sigma = 0.3
#
#     # Initialize cost to 0
#     w = 0
#
#     if intersects(p):
#         w+=5
#
#     # # Gaussian function to apply cost near bumps
#     # for pos in build_pos:
#     #     # print(pos)
#     #     w += 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (np.linalg.norm(pos - p[:2])) ** 2 / (2 * sigma ** 2)) * 1
#     w +=0.1
#     return w
#
# # Get dynamic cost accouting with known objects in time
# def get_dynamic_w(p,t):
#     w = 0
#     sigma = 0.3     # Strandard deviation
#
#     # For each drone position at the given timestep, add gaussian function to apply cost near drones
#     for pos in env.pos_drones[t]:
#         w += 1 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (np.linalg.norm(pos - p)) ** 2 / (2 * sigma ** 2)) * 200
#
#     # Add extra weight (otherwise it would not care about finding shorter paths, as all weights would be 0 in locations with no extra cost)
#     w += 0.01
#     return w
#
# place_name = 'Kamppi, Helsinki, Finland'
#
# center = [24.936287282056615,60.16834436268417]
# build_pos = []
#
# def spheric_to_cartesian(coor):
#     cartesian = np.array([0.,0.])
#     r_Earth = 6378100
#     cartesian[0] = r_Earth*math.sin((coor.x-center[0])*2*math.pi/360)/50
#     cartesian[1] = r_Earth*math.sin((coor.y-center[1])*2*math.pi/360)/50
#     return list(cartesian)
#
# def spheric_to_cartesian2(coor):
#     cartesian = np.array([0.,0.])
#     r_Earth = 6378100
#     cartesian[0] = r_Earth*math.sin((coor[0]-center[0])*2*math.pi/360)/50
#     cartesian[1] = r_Earth*math.sin((coor[1]-center[1])*2*math.pi/360)/50
#     return list(cartesian)
#
# plt.show()
# buildings = ox.footprints_from_place(place_name)['geometry']
# polygons = []
# polygons_obj = []
# lst = np.array(buildings)
# for i in lst:
#     map_poly = mapping(i)
#     coors = np.array(map_poly['coordinates'])
#     print( coors.ndim,map_poly)
#
#     points_list = []
#     if coors.ndim == 4 or coors.ndim == 2:
#         for c2 in coors[0]:
#             for c1 in c2:
#                 pt = spheric_to_cartesian2(np.array(c1))
#                 points_list.append(pt)
#
#     if coors.ndim == 3:
#         for c1 in coors[0]:
#             pt = spheric_to_cartesian2(c1)
#             points_list.append(pt)
#
#     print(points_list)
#
#     polygons_obj.append(Polygon([(p[0],p[1]) for p in points_list]))
#     polygons.append(points_list)
#
#     #
#     # point_collection = MultiPoint(list(points_list))
#     # point_collection.envelope
#     # convex_hull_polygon = point_collection.convex_hull
#     #
#     # map_poly = mapping(convex_hull_polygon)
#     # print(map_poly)
#     # coors_bound = np.array(map_poly['coordinates'])
#     # points_list_bound = []
#     #
#     # if coors_bound.ndim == 4:
#     #     for c2 in coors_bound[0]:
#     #         for c1 in c2:
#     #             points_list_bound.append(np.array(c1))
#     #
#     # if coors_bound.ndim == 3:
#     #     for c1 in coors_bound[0]:
#     #         points_list_bound.append(np.array(c1))
#     # polygons.append(points_list_bound)
#     #
# outFile = open('output.xml', 'wb')
#
# with open('data.txt', 'w') as outfile:
#     json.dump(polygons, outfile)
#
#
# with open('polygons.data', 'r') as f:
#     new_data = pickle.load(f)
# print(np.array(polygons))
# np.savetxt('polygons.out', np.array(polygons))   # X is an array
# # print(len(build_pos))


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


weights = np.loadtxt(fname='weights200.out', delimiter=',')
weights =  weights + np.ones((200,200))

# # Create cost grid (map) (only spatial, no time)
# weights = np.zeros((env.n, env.n))
# for i in range(len(xs)):
#     for j in range(len(ys)):
#         w = get_w(np.array([xs[i], ys[j]]))
#         weights[i, j] = w
