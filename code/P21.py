import numpy as np
import json
import pygame as pg
import time
import random
import itertools
from scipy.optimize import minimize
from shapely import geometry


class Robot():

    def __init__(self, position, goal_position, v_max, index, tau = 2):
        self.p = position
        self.p_goal = goal_position
        self.v = np.zeros(2)
        self.v_opt = self.v
        self.v_max = v_max
        self.index = index # might not be necessary
        self.radius = 0.5
        self.tau = tau
        self.dt = 0.1
        self.v_pref=None
        self.set_v_pref()


    def move(self):
        """Update position"""
        self.p += self.dt*self.v
        self.v_opt=self.v
        self.set_v_pref()

    def select_vel(self, robots):
        """linear programming stuff"""
        ORCA = self.compute_ORCA(robots)
        if ORCA:
            objective = lambda v: np.linalg.norm(v - self.v_pref)

            constraint1 = lambda v, u: np.dot(v - (self.v_opt + 1/2 * u), u/np.linalg.norm(u))
            constraint2 = lambda v: self.v_max - np.linalg.norm(v)


            constraints = ({'type':'ineq', 'fun':constraint2},)
            for u in ORCA:
                constraints += ({'type':'ineq', 'fun':constraint1, 'args':(u,)},)

            v_guess = np.zeros(2)
            res =  minimize(objective, v_guess, method='SLSQP', constraints=constraints, tol=1e-15)
            if np.linalg.norm(res.x)> self.v_max:
                a = 0
            if res.success:
                self.v = res.x
            else:
                self.v = -self.v_opt

        else:

            self.v=self.v_pref



    def compute_ORCA(self, robots):
        """ORCA is defined by u. Returns an array with all u's as rows."""
        ORCA = []
        for robot in robots:
            if robot.index != self.index:
                u = self.compute_u(robot)
                if u.size > 0:
                    ORCA.append(u)

        return ORCA

    def compute_u(self, robot):

        VO = self.create_VO(robot)
        v_opt_rel = self.v_opt - robot.v_opt

        if geometry.Polygon([VO['A'], VO['B'], VO['C'], VO['D']]).contains(geometry.Point(v_opt_rel)):
            x_1 = VO['C']
            x_2 = VO['D']
            u_1 = np.dot(x_1, v_opt_rel)/(np.dot(x_1, x_1))*x_1 - v_opt_rel
            u_2 = np.dot(x_2, v_opt_rel)/(np.dot(x_2, x_2))*x_2 - v_opt_rel

            return u_1 if np.linalg.norm(u_1) < np.linalg.norm(u_2) else u_2

        elif geometry.Point(VO['center']).buffer(VO['radius']).contains(geometry.Point(v_opt_rel)):
            center = VO['center']
            r = VO['radius']
            a = (v_opt_rel - center)*r/np.linalg.norm( v_opt_rel- center)

            return a - v_opt_rel

        else:
            d1=geometry.Polygon([VO['A'], VO['B'], VO['C'], VO['D']]).distance(geometry.Point(v_opt_rel))
            d2=geometry.Point(VO['center']).buffer(VO['radius']).distance(geometry.Point(v_opt_rel))
            if d2 < d1:
                u=(VO['center']-v_opt_rel)/np.linalg.norm(VO['center']-v_opt_rel)*d2
                return u
            else:
                lines=geometry.Polygon([VO['A'], VO['B'], VO['C'], VO['D']]).boundary
                a=0


        return np.array([])

    def create_VO(self, robot):
        """Creating the velocity obstacle"""

        center = (robot.p - self.p)/self.tau
        r = (self.radius + robot.radius)/self.tau
        sh_circle = geometry.Point(center).buffer(r)

        d = np.linalg.norm(center)
        if np.abs(r/d)>1:
            a=0
        theta = np.arccos(r / d)
        phi = np.arctan2(center[1], center[0])
        A_x = r * np.cos(np.pi + theta + phi)
        A_y = r * np.sin(np.pi + theta + phi)
        A = center + np.array([A_x, A_y])
        B_x = r * np.cos(np.pi - theta + phi)
        B_y = r * np.sin(np.pi - theta + phi)
        B = center + np.array([B_x, B_y])
        C = B / np.linalg.norm(B) * 3 * self.v_max  # 2 v_max is as large as it gets, but we want a margin. See drawing.
        D = A / np.linalg.norm(A) * 3 * self.v_max

        return {'center':center, 'radius':r, 'A':A, 'B':B, 'C':C, 'D':D} # C and D are the furthest points in the polygon

    def set_v_pref(self):
        self.v_pref=(self.p_goal-self.p)/self.dt
        if np.linalg.norm(self.v_pref):
            self.v_pref=self.v_pref/np.linalg.norm(self.v_pref)*self.v_max



def colors(n):
    col=list(itertools.permutations([255,0,100,225],3))
    red=(255,0,0)
    orange=(255,100,0)
    green=(0,255,0)
    light_blue=(0,255,255)
    blue=(0,0,255)
    purple=(100,0,255)
    pink=(225,0,255)
    #col=[red,orange,green,light_blue,blue,purple,pink]
    #todo:deal with more than 7 robots
    ret = []
    r =255# int(random.random() * 256)
    g =0# int(random.random() * 256)
    b =255# int(random.random() * 256)
    step = 256 / n
    for i in range(n):
        r += step - int(random.random() * step)
        g += step - int(random.random() * step)
        b += step - int(random.random() * step)
        r = int(r) % 256
        g = int(g) % 256
        b = int(b) % 256
        ret.append(random.choice(col))

    return ret

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
    for r in range(len(robots)):
        pg.draw.circle(screen,agents_colors[r],to_pygame(robots[r].p),7,0)
        pg.draw.circle(screen, agents_colors[r], to_pygame(robots[r].p_goal),7, 1)



def to_pygame(coords):
    '''Convert coordinates into pygame coordinates'''
    return (int(coords[0] * 14 + width / 2 - 150), int(coords[1] * -14 + height / 2 + 200))

# PyGame parameters
pg.init()
infoObject = pg.display.Info()
screen = pg.display.set_mode((infoObject.current_w, infoObject.current_h))
width = infoObject.current_w
height = infoObject.current_h
background_colour = (255, 255, 255)
screen.fill(background_colour)


data = json.load(open('P21.json'))
bounding_polygon = data["bounding_polygon"]
goal_positions=np.array(data["goal_positions"])
start_positions=np.array(data["start_positions"])
vehicle_L = data["vehicle_L"]
vehicle_a_max = data["vehicle_a_max"]
vehicle_omega_max = data["vehicle_omega_max"]
vehicle_phi_max = data["vehicle_phi_max"]
vehicle_t = data["vehicle_t"]
vehicle_v_max = data["vehicle_v_max"]
vehicle_dt=data["vehicle_dt"]
#load obstacles
obstacles=[]
#sh_obstacles=[]
for d in data:
    if "obstacle" in d:
        obstacles.append(data[d])
        #sh_obstacles.append(Polygon(data[d])) todo:maybe use shapely

'''convert obstacles and boundaries to pygame'''
pg_obstacles=[]
for obstacle in obstacles:
    pg_obstacles.append(list_to_pygame(obstacle))

pg_bounding_polygon=list_to_pygame(bounding_polygon)
agents_colors=colors(len(start_positions))

'''Initialization'''
robots = []
for i in range(len(start_positions)):
    robot = Robot(start_positions[i], goal_positions[i], vehicle_v_max, i)
    robots.append(robot)

time_step=0
set_bg()
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
        while (t1 - t0 < 1):
            t1 = time.time()
        start = True

    for robot in robots:
        robot.select_vel(robots)
        robot.move()

    '''for robot in robots:
        robot.move()'''


    set_bg()
    #set_data(total_time * 0.1)
    pg.display.flip()