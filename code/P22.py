import numpy as np
import json
import pyvisgraph as vg
import pygame as pg
import time

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

points_of_interest=data["points_of_interest"]
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

for graph_edge in g.visgraph.edges:

    p1 = graph_edge.p1; p2 = graph_edge.p2
    edge = [p1, p2]
    pg_edges.append(list_to_pygame(edge))

a = 0


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