import numpy as np
import matplotlib.pyplot as plt
import json
import pygame as pg
import time


class Virtual_structure():

    def __init__(self):
        self.center = None
        self.desired_pos = None

    def create_center(self, formation_matrix):
        pass

    def create_des_pos(self):
        pass


class Robot():

    def __init__(self):
        self.location = None
        self.velocity = None

    def control(self):
        # give input signal
        pass


def set_bg(positions):
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    for pos in positions:
        pg_pos = to_pygame(pos)
        pg.draw.circle(screen, (0, 255, 0), (pg_pos[0], pg_pos[1]), 3, 0)
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
bounding_polygon = data["bounding_polygon"]
formation_positions = data["formation_positions"]
start_positions = data["start_positions"]
vehicle_L = data["vehicle_L"]
vehicle_a_max = data["vehicle_a_max"]
vehicle_omega_max = data["vehicle_omega_max"]
vehicle_phi_max = data["vehicle_phi_max"]
vehicle_t = data["vehicle_t"]
vehicle_v_max = data["vehicle_v_max"]

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
    set_bg(start_positions)
    pg.display.flip()