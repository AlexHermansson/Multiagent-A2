import triangle as tr
import triangle.plot as tr_plt
import json
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import *
import pyvisgraph as vg
from P22 import VRP_GA

from shapely import ops
from shapely.ops import triangulate


def polygon_holes(obstacles, sh_obstacles):
    '''function that finds inner points of the given polygons'''
    '''
        input: list of obstacles, each obstacle is a numpy matrix containing the (x,y) coordinates of the vertices
        output: list of points contained in each obstacle, one per obstacle in the order of the obstacles
        *** shapely library is used ***
    '''

    inner_points = []
    for sh_obst in sh_obstacles:
        repr_point = np.array(sh_obst.representative_point().coords)
        inner_points.append(repr_point)

    return np.array(inner_points)

def to_vertices(bounding_polygon, obstacles):
    """A function to take all vertices of the bounding polygon and obstacles
    and stacks them in one large array."""
    vertices = np.array(bounding_polygon)
    for obst in obstacles:
        np_obst = np.array(obst)
        vertices = np.vstack((vertices, np_obst))

    return vertices

def poly_to_segments(bound_poly, obs):
    """Function to create the edges of the bounding polygon and obstacles.
    The edges consists of indices of the vertices. Used in the triangulation."""

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

def visible_triangles(point, sh_obst, radius, tri_list):
    """For an input point, save all visible triangles and a counter of how many that are visible."""

    counter = 0
    triangles_list = []
    sh_point = Point(point)
    for triangle in tri_list:

        triangle_bool = True
        sh_triangle=Polygon(triangle)

        '''check if point can reach the whole triangle'''
        for vertex in triangle:
            if np.linalg.norm(point - vertex) > radius:
                # only breaks inner 'vertex' loop
                triangle_bool = False
                break

        if not triangle_bool:
            continue

        '''check if point is in the triangle'''
        if sh_triangle.crosses(sh_point) or sh_triangle.touches(sh_point):
            counter += 1
            triangles_list.append(triangle)
            continue

        '''check if the point can see every edge of the triangle'''

        sh_tri1=Polygon([triangle[0],triangle[1],point])
        sh_tri2 = Polygon([triangle[1],triangle[2],point])
        sh_tri3 = Polygon([triangle[0],triangle[2],point])
        for obstacle in sh_obst:
            if (sh_tri1.intersects(obstacle) and not sh_tri1.touches(obstacle)) or \
                    (sh_tri2.intersects(obstacle) and not sh_tri2.touches(obstacle))or \
                    (sh_tri3.intersects(obstacle) and not sh_tri3.touches(obstacle)):
                triangle_bool=False
                break

        # If "valid" triangle, increment counter and add to list
        if triangle_bool:
            counter += 1
            triangles_list.append(triangle)


    return counter, triangles_list

def remove_triangles_seen_from_start_and_goal(start_pos, goal_pos, sh_obstacles, sensor_range, triangles):

    point_list=[]

    for pos in start_pos:
        counter, triangles_list = visible_triangles(pos, sh_obstacles, sensor_range, triangles)
        point_list.append(Cluster(pos, counter, triangles_list))

    for pos in goal_pos:
        counter, triangles_list = visible_triangles(pos, sh_obstacles, sensor_range, triangles)
        point_list.append(Cluster(pos, counter, triangles_list))

    remaining_triangles = remove_triangles(triangles, point_list)
    return remaining_triangles

def remove_triangles(tri_list, clusters):

    for cluster in clusters:
        for triangle in cluster.triangles:

            for i, t in enumerate(tri_list):
                if (triangle == t).all():
                    tri_list.pop(i)
                    break


    return tri_list

def remove_best_cluster(triangles, sh_obstacles, vertices, sensor_range):
    point_list = []
    # inner vertices
    for vertex in vertices:
        counter, triangles_list = visible_triangles(vertex, sh_obstacles, sensor_range, triangles)
        point_list.append(Cluster(vertex, counter, triangles_list))

    best_count = 0
    index = 0
    for i,cluster in enumerate(point_list):
        count = cluster.count

        if count == 0:
            vertices = np.delete(vertices, index, axis = 0)
            continue

        if count > best_count:
            best_index = i
            best_count = count
        index += 1

    best_cluster = point_list[best_index]
    triangles = remove_triangles(triangles, [best_cluster])

    return best_cluster.coords, triangles, vertices

def greedy_set_cover(triangles, sh_obstacles, vertices, sensor_range):

    points_to_visit = []
    while triangles:
        point, triangles, vertices = remove_best_cluster(triangles, sh_obstacles, vertices, sensor_range)
        points_to_visit.append(point)


    return points_to_visit

def set_distances(points1, points2, graph):
    """ The distances between two sets of points.
    First argument should be start or goal."""
    K = len(points1)
    N = len(points2)

    D = np.zeros((K, N))
    for i in range(K):
        for j in range(N):
            shortest_path = graph.shortest_path(vg.Point(points1[i][0], points1[i][1]), vg.Point(points2[j][0], points2[j][1]))
            D[i, j] = path_to_distance(shortest_path)

    return D

def path_to_distance(path):
    """Assume path is given as on VG object form."""
    dist = 0
    for i in range(1, len(path)):
        dist += np.linalg.norm(to_np(path[i - 1]) - to_np(path[i]))
    return dist

def to_np(point):
    """From VG point to ndarray"""
    x = point.x
    y = point.y
    return np.array([x, y])

def point_distances(points, graph):
    N = len(points)
    D = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            if i > j:
                shortest_path = graph.shortest_path(vg.Point(points[i][0], points[i][1]), vg.Point(points[j][0], points[j][1]))
                D[i, j] = path_to_distance(shortest_path)

    D += D.T

    return D



'''Loading the data from json'''
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

sh_obstacles = []
for obstacle in obstacles:
    sh_obstacles.append(Polygon(obstacle))

'''Create the triangulation of the map.'''
vertices = to_vertices(bounding_polygon, obstacles)
obst_vertices = vertices[len(bounding_polygon):]
holes = polygon_holes(obstacles, sh_obstacles)
segments = poly_to_segments(bounding_polygon, obstacles)
map_dict = {'vertices':vertices, 'holes':holes, 'segments':segments}
t = tr.triangulate(map_dict, 'p')

visualize_triangulation(t)
'''Remove all the triangle regions that can be seen from the start and goal positions.'''
triangles = triangles_to_list(t)

triangles = remove_triangles_seen_from_start_and_goal(start_positions, goal_positions, sh_obstacles, sensor_range, triangles)



points_to_visit = greedy_set_cover(triangles, sh_obstacles, obst_vertices, sensor_range)
#triangles, vertices = remove_best_cluster(triangles, sh_obstacles, obstacles, sensor_range)

'''create visibility graph'''
'''vg_obstacles=[]
for obstacle in obstacles:
    vg_obstacles.append([vg.Point(p[0],p[1]) for p in obstacle])

g=vg.VisGraph()
g.build(vg_obstacles)
a = [edge for edge in g.visgraph.edges]
p1 = a[0].p1
pg_edges=[]

for edge in g.visgraph.edges:
    p1=[edge.p1.x,edge.p1.y]
    p2=[edge.p2.x,edge.p2.y]
    #pg_edges.append(list_to_pygame([p1,p2]))

D_sp = set_distances(start_positions, points_to_visit, g)
D_pp = point_distances(points_to_visit, g)
D_pg = set_distances(goal_positions, points_to_visit, g)
D_sg = set_distances(start_positions, goal_positions, g)
np.save('D_sp_24', D_sp)
np.save('D_pp_24', D_pp)
np.save('D_pg_24', D_pg)
np.save('D_sg_24', D_sg)'''

D_sp = np.load('D_sp_24.npy')
D_pp = np.load('D_pp_24.npy')
D_pg = np.load('D_pg_24.npy')
D_sg = np.load('D_sg_24.npy')

N = len(points_to_visit) # number of pickup points
k = len(start_positions) # number of robots
pop_size = 1000
generations = 100
#n_trials=30
#N = 5# number of pickup points
#k = 3 # number of robots
lambd=6
vrp_ga = VRP_GA(N, k, D_pg, D_pp, D_sp, D_sg, pop_size,lambd,goal_positions)

vrp_ga.genetic_algorithm(generations,True, 0.01)
plt.plot(vrp_ga.best_scores)
plt.plot(vrp_ga.generation_scores)
plt.xlabel('epoch')
plt.ylabel('objective')
plt.show()
print(vrp_ga.best_score)


#todo: sample points