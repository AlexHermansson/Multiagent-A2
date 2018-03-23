import triangle as tr
import triangle.plot as tr_plt
import json
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import *
from shapely.ops import *
from shapely.ops import triangulate


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
    for point in range(len(bound_poly)-1):
        segments.append(np.array((i,i+1)))
        i+=1
    segments.append(np.array((i, initial)))
    i+=1

    for obstacle in obs:
        initial=i
        for point in range(len(obstacle)-1):
            segments.append(np.array((i, i + 1)))
            i += 1
        segments.append(np.array((i, initial)))
        i+=1

    return np.asarray(segments)

def visualize_triangulation(t):
    ax = plt.subplot()
    tr_plt.plot(ax, **t)
    plt.show()

def triangles_to_list(t):
    """Takes the triangulated object from library, and turn it into a list of triangles."""
    triangles = []
    for indices in t['triangles']:

        triangle = []
        for index in indices:
            triangle.append(np.array(t['vertices'][index]))
        triangles.append(np.array(triangle))

    return triangles

class Cluster():
    """A class for point clusters."""
    def __init__(self, coords, count, triangles):
        self.coords = coords
        self.count = count
        self.triangles = triangles

def visible_triangles(point, polygon_with_holes, radius, tri_list):
    """For an input point, save all visible triangles and a counter of how many that are visible."""

    counter = 0
    triangles_list = []
    sh_point = Point(point)
    for triangle in tri_list:

        triangle_bool = True
        for vertex in triangle:

            if np.linalg.norm(point - vertex) > radius:
                # only breaks inner 'vertex' loop
                triangle_bool = False
                break

        if triangle_bool:
            for vertex in triangle:
                line = LineString([sh_point, Point(vertex)])
                line.s
                # if the line between point and vertex intersects any holes, break
                #if polygon_with_holes.exterior.contains():
                a=line.intersection(polygon_with_holes.exterior)
                if polygon_with_holes.exterior.disjoint(line):
                    triangle_bool = False
                    break

        # If "valid" triangle, increment counter and add to list
        if triangle_bool:
            counter += 1
            triangles_list.append(triangle)


    return counter, triangles_list



'''Loading the data from json'''
data = json.load(open('box.json'))

bounding_polygon = data["bounding_polygon"]
'''goal_positions=np.array(data["goal_positions"])
start_positions=np.array(data["start_positions"])
points_of_interest = np.array(data['points_of_interest'])
vehicle_L = data["vehicle_L"]
vehicle_a_max = data["vehicle_a_max"]
vehicle_omega_max = data["vehicle_omega_max"]
vehicle_phi_max = data["vehicle_phi_max"]
vehicle_t = data["vehicle_t"]
vehicle_v_max = data["vehicle_v_max"]
vehicle_dt=data["vehicle_dt"]'''
sensor_range=data["sensor_range"]

#load obstacles
obstacles=[]
#sh_obstacles=[]
for d in data:
    if "obstacle" in d:
        obstacles.append(np.array((data[d])))

poly_with_holes=Polygon(bounding_polygon,[o for o in obstacles])

vertices = to_vertices(bounding_polygon, obstacles)
holes = polygon_holes(obstacles)
segments = poly_to_segments(bounding_polygon, obstacles)

map_dict = {'vertices':vertices, 'holes':holes, 'segments':segments}
t = tr.triangulate(map_dict, 'p')
#visualize_triangulation(t)
triangles = triangles_to_list(t)


point = np.array([0,0])
counter, triangles_list = visible_triangles(point, poly_with_holes, sensor_range, triangles)

a = 0