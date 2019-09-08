#-------------------------------#
# Author:   Max Martinez Ruts
# Creation: 2019
# Description: All helpers to construct the cost map of the environment, including the modeling of the environment
#-------------------------------#

# Import other scripts
import numpy as np
from environment import env
import helpers as hp
import osmnx as ox
import math
from shapely.geometry import Polygon, MultiPolygon, mapping, Point, MultiPoint
import matplotlib.pyplot as plt
import json
import pickle

# Load list of polygons saved in file
with open('Model/data_city_100.txt') as json_file:
    polygons = json.load(json_file)
polygons_obj = [Polygon([(p[0], p[1]) for p in polygon]) for polygon in polygons]

# Check if point is inside a polygon (building)
def intersects(p):
    res = False
    for pol in polygons_obj:
        if pol.contains(Point(p[0],p[1])):
            res = True
    return res

# Get static cost given a point in space
def get_w(p):
    # Initialize cost to 0
    w = 0

    # If node inside a polygon increase cost
    if intersects(p):
        w+=5

    w +=0.1
    return w

# Get dynamic cost accouting with known objects in time
def get_neighbour_cost(st, en, t):
    w = 0

    # for i in range(len(env.pos_drones[t])):
    #     angle = hp.angle(en-st,env.dir_drones[t][i])
    #     radius = min(angle/30, 3.14/10)
    #     # radius = max(radius, 0.3)
    #     # radius = 0.2
    #     if radius > np.linalg.norm(en-env.pos_drones[t][i]) or radius > np.linalg.norm(st-env.pos_drones[t][i]):
    #         if t>30:
    #             w+=100000000000000

    tot_dir = np.array([0.0, 0.0])
    tot_pos = np.array([0.0,0.0])
    k = 0
    for i in range(len(env.dir_drones[t])):
        if np.linalg.norm(env.pos_drones[t][i][:2] - (st+en) / 2) < 0.25:
            tot_dir += env.dir_drones[t][i][:2]
            tot_pos += env.pos_drones[t][i][:2]
            k += 1
    if k > 0:
        tot_dir /= k
        tot_pos /= k

        w += hp.angle(en - st, tot_dir)**3/k
        w += np.linalg.norm(tot_pos-(en+st)/2)*25/k

    if k == 0:
        w += 100
    w+=0.000001
    return max(1, w)

def get_separation_cost(st, en, t):
    w= 0
    for i in range(len(env.pos_drones[t])):
        angle = hp.angle(en[:2]-st[:2],env.dir_drones[t][i][:2])
        angle_vs = hp.angle(en[:2]-st[:2],env.pos_drones[t][i][:2]-st[:2])
        radius = min((3*angle)**2/2,7)
        if radius > abs((en[2]+st[2])/2-env.pos_drones[t][i][2]) and radius/13 > np.linalg.norm((en[:2]+st[:2])/2-env.pos_drones[t][i][:2]) and angle_vs<math.pi/4:
            if t>30 and angle > 1:
                w+=30000000



    tot_pos = np.array([0.0, 0.0, 0.0])
    k = 0
    for i in range(len(env.dir_drones[t])):
        if np.linalg.norm(env.pos_drones[t][i][:2] - (st[:2] + en[:2]) / 2) < 1:
            tot_pos += env.pos_drones[t][i]
            k += 1
    if k > 0:
        tot_pos /= k

        w += np.linalg.norm(tot_pos - (en + st) / 2) * 25 / k

    if k == 0:
        w += 100

    w+=0.000001
    return max(1.00001, w)
def get_alignment_cost(dir, t):
    w = 0
# def get_dynamic_w(pos, t):
#     w = 0
#     sigma = 0.3     # Strandard deviation
#
#     #        w += 2 / (sigma * np.sqrt(2 * np.pi)) * np.exp(- (np.linalg.norm(pos - env.pos_drones[t][i])) ** 2 / (2 * sigma ** 2))
#
#
#     perception_radius = 1
#     percepted_pos = []
#
#     # For each drone position at the given timestep, add gaussian function to apply cost near drones
#     for i in range(len(env.pos_drones[t])):
#         if np.linalg.norm(pos-env.pos_drones[t][i])<perception_radius:
#             percepted_pos.append(env.pos_drones[t][i])
#
#     percepted_pos = np.array(percepted_pos)
#
#     tot =  np.array([0.0,0.0])
#     for p in percepted_pos:
#         tot += p
#     if len(percepted_pos)>0:
#         mean_pos = tot /len(percepted_pos)
#     else: mean_pos = np.array(pos)
#     # print(tot,'mean' )
#     # Alignment
#
#     # Separation
#     for p in percepted_pos:
#         w+=1
#
#     # Cohesion
#     # w = np.linalg.norm(pos-mean_pos)
#
#     # Add extra weight (otherwise it would not care about finding shorter paths, as all weights would be 0 in locations with no extra cost)
#     return w
# #
place_name = 'Kamppi, Helsinki, Finland'
#
# center = [24.931042,60.166594]
# build_pos = []
#
# def spheric_to_cartesian(coor):
#     cartesian = np.array([0.,0.])
#     r_Earth = 6378100
#
#     # Set x coordinate
#     cartesian[0] = r_Earth*math.sin((coor.x-center[0])*2*math.pi/360)/50*math.cos(coor.y*2*math.pi/360)
#
#     # Set y coordinate
#     cartesian[1] = r_Earth*math.sin((coor.y-center[1])*2*math.pi/360)/50
#
#     return list(cartesian)
#
# def spheric_to_cartesian2(coor):
#     cartesian = np.array([0.,0.])
#     r_Earth = 6378100
#
#     # Set x coordinate
#     cartesian[0] = r_Earth*math.sin((coor[0]-center[0])*2*math.pi/360)/50*math.cos(coor[1]*2*math.pi/360)
#
#     # Set y coordinate
#     cartesian[1] = r_Earth*math.sin((coor[1]-center[1])*2*math.pi/360)/50
#
#     return list(cartesian)
#
# plt.show()
#
# # Load environment
# buildings = ox.footprints_from_place(place_name)
#
# # Get geometry subset (buildings)
# geometries = buildings['geometry']
# geometries = np.array(geometries)
#
# # Print heights of buildings
# heights = buildings['height']
# heights = np.array(heights)
#
# # Initialize list of polygons
# polygons = []
# polygons_obj = []
#
# # For each building
# for i in range(len(np.array(buildings))):
#     map_poly = mapping(geometries[i])
#     height = heights[i]
#     coors = np.array(map_poly['coordinates'])
#
#     # Get list of points for each vertex and convert it to cartesian coordinates
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
#     # Construct polygon
#     polygons_obj.append(Polygon([(p[0],p[1]) for p in points_list]))
#     polygons.append(points_list)
#
# point_collection = MultiPoint(list(points_list))
# point_collection.envelope
# convex_hull_polygon = point_collection.convex_hull
#
# map_poly = mapping(convex_hull_polygon)
# coors_bound = np.array(map_poly['coordinates'])
# points_list_bound = []
#
# if coors_bound.ndim == 4:
#     for c2 in coors_bound[0]:
#         for c1 in c2:
#             points_list_bound.append(np.array(c1))
#
# if coors_bound.ndim == 3:
#     for c1 in coors_bound[0]:
#         points_list_bound.append(np.array(c1))
# polygons.append(points_list_bound)
#
#
#
#
#
#
#
# outFile = open('Model/output_city.xml', 'wb')
# print('------')
# class NumpyEncoder(json.JSONEncoder):
#     def default(self, obj):
#         if isinstance(obj, np.ndarray):
#             return obj.tolist()
#         return json.JSONEncoder.default(self, obj)
# print(polygons_obj)
# print(json.dumps(polygons, cls=NumpyEncoder))
# with open('Model/data_city_100.txt', 'w') as outfile:
#     json.dump(polygons,outfile, cls=NumpyEncoder)
#
#
# with open('polygons.data', 'r') as f:
#     new_data = pickle.load(str(f))
# print(np.array(polygons))
# np.savetxt('Model/polygons_100.out', np.array(polygons),fmt='%s')   # X is an array
# print(len(build_pos))

#
# Create spatial grid
xs = np.linspace(-8, 8, env.n)
ys = np.linspace(-8, 8, env.n)
zs = np.linspace( 0, 20, env.m)

# Use seed to generate same results for each simulation even using random events

# Generate bumps at random positions to illustrate the concept of cost map
# bumps = []
# for i in np.linspace(-6,6,9):
#     for j in np.linspace(-6,6,9):
#         bumps.append([i,j])
# bumps = np.array(bumps)
# bumps = np.random.uniform(-6,6,(50,2))

# weights = np.ones((80,80))
weights = np.loadtxt(fname='Model/weights100.out', delimiter=',')
weights =  weights + np.ones((100,100))
#
# # # Create cost grid (map) (only spatial, no time)
# weights = np.zeros((env.n, env.n, env.m))
# for i in range(len(xs)):
#     for j in range(len(ys)):
#         w = get_w(np.array([xs[i], ys[j]]))
#         for k in range(len(zs)):
#             weights[i, j, k] = w
# np.save('Model/weights100_3d.py', weights)   # X is an array
