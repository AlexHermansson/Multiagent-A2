import json
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import *
import pyvisgraph as vg
import pygame as pg
import time
import random
from numpy.linalg import norm


class Cluster():

    def __init__(self,coords):
        self.coords=coords
        self.neighbours=[]
        self.seen=False
        self.count=0


'''Loading the data from json'''
data = json.load(open('P23.json'))

bounding_polygon = data["bounding_polygon"]
goal_positions=np.array(data["goal_positions"])
start_positions=np.array(data["start_positions"])
points_of_interest = np.array(data['points_of_interest'])
vehicle_L = data["vehicle_L"]
vehicle_a_max = data["vehicle_a_max"]
vehicle_omega_max = data["vehicle_omega_max"]
vehicle_phi_max = data["vehicle_phi_max"]
vehicle_t = data["vehicle_t"]
vehicle_v_max = data["vehicle_v_max"]
vehicle_dt=data["vehicle_dt"]
sensor_range=data["sensor_range"]

#load obstacles
obstacles=[]
#sh_obstacles=[]
for d in data:
    if "obstacle" in d:
        obstacles.append(np.array((data[d])))

sh_obstacles = []
for obstacle in obstacles:
    sh_obstacles.append(Polygon(obstacle))


sh_bounding_polygon = Polygon(bounding_polygon)
min_x, min_y, max_x, max_y = sh_bounding_polygon.bounds

a = 0

cluster_list=[]

for point in points_of_interest:
    cluster_list.append(Cluster(point))


for i,this_cluster in enumerate(cluster_list):
    for other_cluster in cluster_list[i+1:]:
        if norm(this_cluster.coords-other_cluster.coords)<sensor_range:
            #todo: check obstacle
            this_cluster.neighbours.append(other_cluster)
            other_cluster.neighbours.append(this_cluster)
            this_cluster.count+=1
            other_cluster.count+=1
