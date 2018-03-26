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

def remove_start_goal(start_points, goal_points, points, sh_obst, range):

    start_goal_list = []

    for start in start_points:
        index = 0
        cluster = Cluster(start)
        for point in points:

            if norm(start - point) < range:
                if visible(start, point, sh_obst):
                    points = np.delete(points, index, axis=0)
                    cluster.neighbours.append(Cluster(point))

                else:
                    index += 1
        start_goal_list.append(cluster)

    for goal in goal_points:
        index = 0
        cluster = Cluster(goal)
        for point in points:

            if norm(goal - point) < range:
                if visible(goal, point, sh_obst):
                    points = np.delete(points, index, axis=0)
                    cluster.neighbours.append(Cluster(point))
                else:
                    index += 1
        start_goal_list.append(cluster)

    return points, start_goal_list

def visible(point1, point2, sh_obst):
    '''Function to check if two points are visible to each other.'''

    line = LineString((Point(point1), Point(point2)))

    vis = True
    for obst in sh_obst:
        if line.intersects(obst):
            vis = False
            break

    return vis

def points_to_clusters(points):
    '''Create list of clusters from points on coordinate form.'''

    cluster_list = []
    for point in points:
        cluster_list.append(Cluster(point))

    return cluster_list

def assign_neighbours(clusters):
    '''Function to assign neighbour clusters to each point of interest'''

    for i,this_cluster in enumerate(clusters):
        for other_cluster in clusters[i+1:]:
            if norm(this_cluster.coords-other_cluster.coords)<sensor_range:
                if visible(this_cluster.coords, other_cluster.coords, sh_obstacles):
                    this_cluster.neighbours.append(other_cluster)
                    other_cluster.neighbours.append(this_cluster)
                    this_cluster.count+=1
                    other_cluster.count+=1

    return clusters

def remove_greedily(clusters):
    '''Remove the best cluster, i.e. greedily with respect to how many neighbours it has.'''

    best_cluster = clusters[0]
    best_count = 0
    for index, cluster in enumerate(clusters):

        if cluster.count > best_count:
            best_cluster = cluster
            best_count = cluster.count

    best_cluster.seen = True
    for neighbour in best_cluster.neighbours:
        neighbour.seen = True
        clusters.remove(neighbour)

    clusters.remove(best_cluster)
    clusters = update_neighbours(clusters)

    return best_cluster, clusters

def update_neighbours(clusters):
    '''Update neighbourhoods of remaining clusters'''

    for cluster in clusters:
        index = 0
        for neighbour in cluster.neighbours:
            if neighbour.seen:
                cluster.neighbours.pop(index)
            else:
                index += 1

        cluster.count = index

    return clusters


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




points_0, start_goal_clusters = remove_start_goal(start_positions, goal_positions,
                                                  points_of_interest, sh_obstacles, sensor_range)

cluster_list = points_to_clusters(points_0)
cluster_list = assign_neighbours(cluster_list)

def find_clusters(clusters):
    '''Remove points from clusters greedily until no more are remaining.'''

    best_clusters = []
    while clusters:

        best_cluster, clusters = remove_greedily(clusters)
        best_clusters.append(best_cluster)

    return best_clusters

best_clusters = find_clusters(cluster_list)

a = 0