import numpy as np
import matplotlib.pyplot as plt
import json
import pygame as pg
import time


class Virtual_structure():
    """"
    A class for the virtual structure with a center point in the variable mean and the orientation.
    It keeps track of the desired positions for all robots.
    """
    def __init__(self):
        self.mean = None
        self.desired_pos = None
        self.D_list = None
        self.A_list = None
        self.orientation = None
        self.omega = 0 # angular velocity
        self.step = 0
        self.velocity=np.array([0,0])


    def set_des_pos(self, new_mean, new_orientation):
        """ A function to update the desired positions for the robots when the mean and orientation is changed."""
        des_pos = []
        for d, a in zip(self.D_list, self.A_list):
            des_pos.append(new_mean + d * np.array([np.cos(a + new_orientation), np.sin(a + new_orientation)]))
        self.mean=new_mean
        if self.step > 1:
            self.omega = (new_orientation - self.orientation)/0.1
        self.orientation=new_orientation
        self.desired_pos = des_pos
        self.step += 1


    def set_formation(self, formation):
        """Creates the form of the virtual structure."""
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

        self.mean = mean
        self.D_list = np.array(dist_list)
        self.A_list = np.array(angle_list)



class Robots():

    #kp = np.diag([10, 10])
    #kv =  np.diag([16, 16])


    def __init__(self, locations,N=7):
        self.locations = locations
        self.N=N
        #self.velocity = np.array([0, 0])
        self.velocities = np.zeros(2*N).reshape(N, 2) # if N robots, we need N velocities
        self.v_max = vehicle_v_max
        self.a_max = vehicle_a_max
        self.kp=np.diag(np.ones(N)*10)
        self.kv=np.diag(np.ones(N)*16)
        self.dt=0.1


    def control(self, vs):
        """A function to give input signal given where the virtual structure, vs, is."""

        # Arclengths that the robots have to move
        s = np.linalg.norm(vs.desired_pos - self.locations)

        z_dot_des_x = vs.velocity[0] - vs.D_list*vs.omega/self.dt*np.sin(vs.orientation + vs.A_list)
        z_dot_des_y = vs.velocity[1] + vs.D_list*vs.omega/self.dt*np.cos(vs.orientation + vs.A_list)
        z_dot_des = np.array([z_dot_des_x, z_dot_des_y]).reshape(self.N,2)
        z_hat_dot = self.velocities - z_dot_des

        # location or acceleration? todo: check that it works
        #u = vs.desired_pos - self.kp.dot(self.locations - vs.desired_pos) - self.kv.dot(z_hat_dot)
        u = -self.kp.dot(self.locations - vs.desired_pos) - self.kv.dot(z_hat_dot)

        # make sure the acceleration is not to large
        if np.linalg.norm(u) > self.a_max:
            u = u/np.linalg.norm(u)

        return u

    def move(self, vs):
        """A function to update the lcoation and velocity for one time step."""

        u = self.control(vs)

        self.locations = 1/2*u*(self.dt)**2 + self.velocities*self.dt + self.locations

        new_vel = u*self.dt + self.velocities
        # make sure the velocity is within the limit v_max
        if np.linalg.norm(new_vel) > self.v_max:
            new_vel = new_vel / np.linalg.norm(new_vel)
        self.velocities = new_vel



def set_bg(positions):
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    for pos in positions:
        pg_pos = to_pygame(pos)
        pg.draw.circle(screen, (0, 255, 0), (pg_pos[0], pg_pos[1]), 3, 0)
    for i in range(1,len(traj_pos)):
        pg.draw.line(screen,(255,0,0),to_pygame(traj_pos[i-1]),to_pygame(traj_pos[i]))
    pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)
    for pos in vs.desired_pos:
        pg_pos = to_pygame(pos)
        pg.draw.circle(screen, (0, 0, 255), (pg_pos[0], pg_pos[1]), 3, 1)
    pg_mean=to_pygame(vs.mean)
    pg.draw.circle(screen, (0, 0, 0), pg_mean, 3, 0)



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
start_positions = np.array(data["start_positions"])
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
traj_pos=np.array(list(zip(traj_x,traj_y)))

pg_bounding_polygon = []
for point in bounding_polygon:
    pg_bounding_polygon.append(to_pygame(point))

'''Initialization'''
vs=Virtual_structure()
vs.set_formation(formation_positions)
vs.set_des_pos(traj_pos[0],traj_theta[0])
robots=Robots(start_positions,start_positions.shape[0])

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
    for t in range(10):
        robots.move(vs)
    set_bg(robots.locations)
    pg.display.flip()


