import json
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import *
import pyvisgraph as vg
import pygame as pg
import time
import random








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