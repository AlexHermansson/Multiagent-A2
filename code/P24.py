import triangle as tr
import triangle.plot as tr_plt
import json
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import *


def polygon_holes(obstacles):
    '''function that finds inner points of the given polygons'''
    '''
        input: list of obstacles, each obstacle is a numpy matrix containing the (x,y) coordinates of the veretxes
        output: list of points contained in each obstacle, one per obstacle in the order of the obstacles
        *** shapely library is used ***
    '''
    sh_obstacles = []
    for obstacle in obstacles:
        sh_obstacles.append(Polygon(obstacle))

    inner_points = []
    for i, obstacle in enumerate(obstacles):
        found = False
        while not found:
            x_min = np.min(obstacle[:, 0])
            x_max = np.max(obstacle[:, 0])
            y_min = np.min(obstacle[:, 1])
            y_max = np.max(obstacle[:, 1])
            rand_x = np.random.uniform(x_min, x_max)
            rand_y = np.random.uniform(y_min, y_max)
            if sh_obstacles[i].contains(Point([rand_x, rand_y])):
                inner_points.append(np.array((rand_x, rand_y)))
                break

    return np.array(inner_points)

def to_vertices(bounding_polygon, obstacles):
    """A function to take all vertices of the bounding polygon and obstacles
    and put them into one large array."""
    vertices = np.array(bounding_polygon)
    for obst in obstacles:
        np_obst = np.array(obst)
        vertices = np.vstack((vertices, np_obst))

    return vertices

def poly_to_segments(bound_poly, obs):

    i=0
    segments=[]
    initial=i
    for point in bound_poly:
        segments.append(np.array((i,i+1)))
        i+=1
    segments.append(np.array((i-1, initial)))

    for obstacle in obs:
        initial=i
        for point in obstacle:
            segments.append(np.array((i, i + 1)))
            i += 1
        segments.append(np.array((i - 1, initial)))

    return np.asarray(segments)



data = json.load(open('P24.json'))

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


face = tr.get_data('face')
spiral = tr.get_data('spiral')

vertices = to_vertices(bounding_polygon, obstacles)
holes = polygon_holes(obstacles)

#plt.scatter(vertices[:,0], vertices[:,1])
#plt.show()

map_dict = {'vertices':vertices, 'holes':holes}
t = tr.triangulate(map_dict)
a = 0


tr_plt.compare(plt, map_dict, t)
plt.show()
