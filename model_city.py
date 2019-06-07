import osmnx as ox
import matplotlib.pyplot as plt
import numpy as np
import math
place_name = 'Kamppi, Helsinki, Finland'

center = [24.936287282056615,60.16834436268417]

def spheric_to_cartesian(coor):
    cartesian = np.array([0.,0.])
    r_Earth = 6378100
    cartesian[0] = r_Earth*math.sin((coor.x-center[0])*2*math.pi/360)/50
    cartesian[1] = r_Earth*math.sin((coor.y-center[1])*2*math.pi/360)/50
    return cartesian


buildings = ox.footprints_from_place(place_name)['geometry']
buildings.plot()
lst = np.array(buildings)
for i in lst:
    print(i.centroid)
    print(spheric_to_cartesian(i.centroid))

    # graph = ox.graph_from_place(place_name)
#area = ox.gdf_from_place(place_name)
#area.plot()
# fig, ax = ox.plot_graph(graph)
# buildings = ox.buildings_from_place(place_name)