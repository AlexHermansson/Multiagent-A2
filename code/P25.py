import numpy as np
import matplotlib.pyplot as plt
import json
import pygame as pg
import time


class Virtual_structure():

    def __init__(self):
        self.mean = None
        self.desired_pos = None
        self.orientation = None
        set_center(formation_matrix)

    def set_center(self, formation_matrix):

        mean_x = np.mean(formation_matrix[0])
        mean_y = np.mean(formation_matrix[1])
        mean = np.array([mean_x, mean_y])
        self.mean = mean

    def set_des_pos(self, formation_matrix):

        pass

def start_pos(formation):

    mean_x = np.mean(formation[:, 0])
    mean_y = np.mean(formation[:, 1])
    mean = np.array([mean_x, mean_y])
    dist_list = []
    angle_list = []
    for point in formation:

        dist = np.linalg.norm(point - mean)
        dist_list.append(dist)
        angle = np.arctan2(point[1] - mean[1], point[0] - mean[0]) - np.pi/2
        if angle < 0:
            angle = 2*np.pi + angle
        angle_list.append(angle)

    return dist_list, angle_list





class Robot():

    def __init__(self, location, center):
        self.location = location
        self.velocity = np.array([0, 0])
        self.center = center
        self.desired_location = center + D * [np.cos(theta), np.sin(theta)]


    def control(self):
        # give input signal
        pass

    def move(self):
        u = control()


def set_bg(positions):
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    for pos in positions:
        pg_pos = to_pygame(pos)
        pg.draw.circle(screen, (0, 255, 0), (pg_pos[0], pg_pos[1]), 3, 0)
    for i in range(1,len(traj_pos)):
        pg.draw.line(screen,(255,0,0),to_pygame(traj_pos[i-1]),to_pygame(traj_pos[i]))
    pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)


def to_pygame(coords):
    '''Convert coordinates into pygame coordinates'''
    return (int(coords[0] * 5 + width / 2 - 150), int(coords[1] * -5 + height / 2 + 200))


pg.init()
infoObject = pg.display.Info()
screen = pg.display.set_mode((infoObject.current_w, infoObject.current_h))
width = infoObject.current_w
height = infoObject.current_h
background_colour = (255, 255, 255)
screen.fill(background_colour)

data = json.load(open('P25.json'))
traj=json.load(open('P25_26_traj.json'))
bounding_polygon = data["bounding_polygon"]
formation_positions = np.array(data["formation_positions"])
start_positions = data["start_positions"]
vehicle_L = data["vehicle_L"]
vehicle_a_max = data["vehicle_a_max"]
vehicle_omega_max = data["vehicle_omega_max"]
vehicle_phi_max = data["vehicle_phi_max"]
vehicle_t = data["vehicle_t"]
vehicle_v_max = data["vehicle_v_max"]
traj_t=traj["t"]
traj_theta=traj["theta"]
traj_x=traj["x"]
traj_y=traj["y"]
traj_pos=list(zip(traj_x,traj_y))



d_list, a_list = start_pos(formation_positions)
start_mean = traj_pos[0]
start_angle = traj_theta[0]

start_pos_list = []
for dist, angle in zip(d_list, a_list):

    pos = start_mean + dist*np.array([np.cos(angle + start_angle), np.sin(angle + start_angle)])
    start_pos_list.append(pos)





pg_bounding_polygon = []
for point in bounding_polygon:
    pg_bounding_polygon.append(to_pygame(point))

set_bg(start_positions)
pg.display.flip()
start = False
done = False
while not done:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            done = True
    if not start:
        t0 = time.time()
        t1 = 0
        while (t1 - t0 < 2):
            t1 = time.time()
        start = True
    set_bg(start_pos_list)
    pg.display.flip()

