import numpy as np
import json
import pyvisgraph as vg
import pygame as pg
import time


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

def list_to_pygame(list_of_points):
    pg_list_of_points=[]
    for point in list_of_points:
        pg_list_of_points.append(to_pygame(point))
    return  pg_list_of_points

def set_bg():
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    '''set obstacles'''
    [pg.draw.polygon(screen, (0, 0, 0), pg_obstacle, 0) for pg_obstacle in pg_obstacles]
    '''set boundaries'''
    pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)
    '''set start and goal positions'''
    for edge in pg_edges:
        pg.draw.line(screen,(0,0,255),edge[0],edge[1])




def to_pygame(coords):
    '''Convert coordinates into pygame coordinates'''
    return (int(coords[0] * 14 + width / 2 - 150), int(coords[1] * -14 + height / 2 + 200))


'''set pygame env'''
pg.init()
infoObject = pg.display.Info()
screen = pg.display.set_mode((infoObject.current_w, infoObject.current_h))
width = infoObject.current_w
height = infoObject.current_h
background_colour = (255, 255, 255)
screen.fill(background_colour)

'''import json data'''
data = json.load(open('P22.json'))
bounding_polygon = data["bounding_polygon"]
goal_positions=np.array(data["goal_positions"])

obstacles=[]
for d in data:
    if "obstacle" in d:
        obstacles.append(data[d])

points_of_interest=np.array(data["points_of_interest"])
start_positions=np.array(data["start_positions"])
vehicle_L = data["vehicle_L"]
vehicle_a_max = data["vehicle_a_max"]
vehicle_dt=data["vehicle_dt"]
ehicle_omega_max = data["vehicle_omega_max"]
vehicle_phi_max = data["vehicle_phi_max"]
vehicle_t = data["vehicle_t"]
vehicle_v_max = data["vehicle_v_max"]

'''set pygame obstacles anf bounding polygon'''
pg_obstacles=[]
for obstacle in obstacles:
    pg_obstacles.append(list_to_pygame(obstacle))

pg_bounding_polygon=list_to_pygame(bounding_polygon)

'''create visibility graph'''
vg_obstacles=[]
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
    pg_edges.append(list_to_pygame([p1,p2]))


#D_sp = set_distances(start_positions, points_of_interest, g)
#D_pp = point_distances(points_of_interest, g)
#D_pg = set_distances(goal_positions, points_of_interest, g)
#D_sg = set_distances(start_positions, goal_positions, g)
#np.save('D_sp', D_sp)
#np.save('D_pp', D_pp)
#np.save('D_pg', D_pg)
#np.save('D_sg', D_sg)

D_sp = np.load('D_sp.npy')
D_pp = np.load('D_pp.npy')
D_pg = np.load('D_pg.npy')
D_sg = np.load('D_sg.npy')





time_step=0
start = False
done = False


while not done:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            done = True
    if not start:
        t0 = time.time()
        t1 = 0
        while (t1 - t0 < 1):
            t1 = time.time()
        start = True

        set_bg()
        # set_data(total_time * 0.1)
        pg.display.flip()