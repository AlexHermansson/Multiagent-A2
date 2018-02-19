import numpy as np
import matplotlib.pyplot as plt
import json
import pygame as pg
import time
import random





class Virtual_structure():
    """"
    A class for the virtual structure with a center point in the variable mean and the orientation.
    It keeps track of the desired positions for all robots.
    """
    def __init__(self,n_robots):
        self.mean = None
        self.desired_pos = None
        self.D_list = None
        self.A_list = None
        self.orientation = None
        self.omega = 0 # angular velocity
        self.tau = 0 # angular acceleration
        self.acceleration = np.zeros(2)
        self.xi_velocity = np.zeros(3 + 2*n_robots).reshape(-1, 1)
        self.xi_acceleration = np.zeros(3 + 2*n_robots).reshape(-1, 1)
        self.xi = None
        self.xi_desired = None
        self.dt = 0.1


    def init_pos(self, new_mean, new_orientation):
        """ A function to update the desired positions for the robots when the mean and orientation is changed."""
        des_pos = []
        for d, a in zip(self.D_list, self.A_list):
            des_pos.append(new_mean + d * np.array([np.cos(a + new_orientation), np.sin(a + new_orientation)]))
        self.mean=new_mean
        self.orientation=new_orientation
        self.desired_pos = np.array(des_pos)
        self.init_xi(new_mean, new_orientation)

    def set_des_xi(self, new_mean, new_orientation):
        self.xi_desired = np.array([new_mean[0], new_mean[1], new_orientation] + [d for d in self.D_list] + [a for a in self.A_list])

    def init_xi(self, new_mean, new_orientation):
        self.xi = np.array([new_mean[0], new_mean[1], new_orientation] + [d for d in self.D_list] + [a for a in self.A_list])

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


    def update_structure(self, robot_positions):
        """Update the velocity for the structure, given the positions of the
        robots and the desired position on the trajectory."""

        K = 5 #0.2 in paper
        K_F = 2
        k_1 = 10 #2.3 in paper
        N = robot_positions.shape[0]
        Z_hat = robot_positions - self.desired_pos
        #phi = 1/N * Z_hat.dot(Z_hat.T)
        phi = 1/N * np.einsum('ij, ij ', Z_hat, Z_hat) # this might work?
        gamma = 1/(K_F*phi + 1/k_1)

        # todo: this won't work? phi is now fixed at least (I think)


        delta_xi = self.xi - self.xi_desired
        old_xi_velocity = self.xi_velocity
        if np.abs(delta_xi[2]) > np.pi:
            delta_xi[2] = -np.sign(delta_xi[2])*(2*np.pi - np.abs(delta_xi[2]))

        self.xi_velocity = -gamma*K*np.tanh(1/K*(delta_xi))

        z_dot_des_x = vs.xi_velocity[0] - vs.D_list*vs.omega*np.sin(vs.orientation + vs.A_list)
        z_dot_des_y = vs.xi_velocity[1] + vs.D_list*vs.omega*np.cos(vs.orientation + vs.A_list)
        Z_hat_dot = (robots.velocities - np.array([z_dot_des_x, z_dot_des_y]).reshape(N, 2))
        phi_dot = 2/N * np.einsum('ij, ij', Z_hat, Z_hat_dot)
        self.xi_acceleration = -gamma*1/np.cosh(1/K*(delta_xi)**2)*self.xi_velocity + 2*gamma**2*K_F/N*phi_dot*K*np.tanh(1/K*delta_xi)

        self.xi = self.xi + self.xi_velocity*self.dt #+ 1/2*self.xi_acceleration*(self.dt)**2
        self.xi_to_structure()


    def xi_to_structure(self):
        self.mean = np.array([self.xi[0], self.xi[1]])
        self.orientation = self.xi[2]
        des_pos = []
        for d, a in zip(self.D_list, self.A_list):
            des_pos.append(self.mean + d * np.array([np.cos(a + self.orientation), np.sin(a + self.orientation)]))
        self.desired_pos = np.array(des_pos)
        self.omega = self.xi_velocity[2]
        self.acceleration = self.xi_acceleration[0:2]
        self.tau = self.xi_acceleration[2]


class Robots():


    def __init__(self, locations,N=7):
        self.locations = locations
        self.N=N
        self.velocities = np.zeros(2*N).reshape(N, 2) # if N robots, we need N velocities
        self.v_max = vehicle_v_max
        self.a_max = vehicle_a_max
        self.kp=np.diag(np.ones(N)*10)
        self.kv=np.diag(np.ones(N)*16)
        self.dt=0.1
        self.colors=colors(N)
        self.all_locations=[]
        self.start=False


    def control(self, vs):
        """A function to give input signal given where the virtual structure, vs, is."""

        # Arclengths that the robots have to move
        s = np.linalg.norm(vs.desired_pos - self.locations)

        # todo: check these formulas
        z_dot_des_x = vs.xi_velocity[0] - vs.D_list*vs.omega*np.sin(vs.orientation + vs.A_list)
        z_dot_des_y = vs.xi_velocity[1] + vs.D_list*vs.omega*np.cos(vs.orientation + vs.A_list)
        z_dot_des = np.array([z_dot_des_x, z_dot_des_y]).reshape(self.N,2)
        z_hat_dot = self.velocities - z_dot_des

        # location or acceleration? todo: check that it works
        #u = vs.desired_pos - self.kp.dot(self.locations - vs.desired_pos) - self.kv.dot(z_hat_dot)
        x_acc = vs.acceleration[0] - vs.D_list*vs.omega**2*np.cos(vs.orientation + vs.A_list) - vs.D_list*vs.tau*np.sin(vs.orientation + vs.A_list)
        y_acc = vs.acceleration[1] - vs.D_list*vs.omega**2*np.sin(vs.orientation + vs.A_list) + vs.D_list*vs.tau*np.cos(vs.orientation + vs.A_list)
        z_acc = np.array([x_acc, y_acc])
        #u = z_acc.T - self.kp.dot(self.locations - vs.desired_pos) - self.kv.dot(z_hat_dot)
        u = - self.kp.dot(self.locations - vs.desired_pos) - self.kv.dot(z_hat_dot)
        # make sure the acceleration is not to large
        u_idx = np.linalg.norm(u, axis=1) > self.a_max



        for i in range(len(u)):
            if np.linalg.norm(u[i]) >self.a_max:
                u[i] = (u[i]*self.a_max)/np.linalg.norm(u[i])
            #u = (u*self.a_max)/np.linalg.norm(u)

        return u

    def move(self, vs):
        """A function to update the lcoation and velocity for one time step."""

        u = self.control(vs)

        self.locations = 1/2*u*(self.dt)**2 + self.velocities*self.dt + self.locations
        if self.start is True:
            self.all_locations.append(self.locations)
            if len(self.all_locations)>500:
                self.all_locations.pop(0)


        new_vel = u*self.dt + self.velocities
        # make sure the velocity is within the limit v_max
        for i in range(len(new_vel)):
            if np.linalg.norm(new_vel[i]) > self.v_max:
                new_vel[i] = (new_vel[i]*self.v_max) / np.linalg.norm(new_vel[i])
            #new_vel = (new_vel*self.v_max) / np.linalg.norm(new_vel)
        self.velocities = new_vel


def colors(n):
    red=(255,0,0)
    orange=(255,100,0)
    green=(0,255,0)
    light_blue=(0,255,255)
    blue=(0,0,255)
    purple=(100,0,255)
    pink=(225,0,255)
    col=[red,orange,green,light_blue,blue,purple,pink]
    #todo:deal with more than 7 robots
    '''ret = []
    r = int(random.random() * 256)
    g = int(random.random() * 256)
    b = int(random.random() * 256)
    step = 256 / n
    for i in range(n):
        r += step
        g += step
        b += step
        r = int(r) % 256
        g = int(g) % 256
        b = int(b) % 256
        ret.append((r, g, b))'''

    return col

def set_bg():
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    for i in range(len(robots.locations)):
        pg_pos = to_pygame(robots.locations[i])
        pg.draw.circle(screen, robots.colors[i], (pg_pos[0], pg_pos[1]), 3, 0)
        if len(robots.all_locations)>1:
            for p in range(1,len(robots.all_locations)):
                pg.draw.line(screen,robots.colors[i],to_pygame(robots.all_locations[p-1][i]),to_pygame(robots.all_locations[p][i]))
    for i in range(1,len(traj_pos)):
        pg.draw.line(screen,(0,0,0),to_pygame(traj_pos[i-1]),to_pygame(traj_pos[i]))
    pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)
    for pos in vs.desired_pos:
        pg_pos = to_pygame(pos)
        pg.draw.circle(screen, (0, 0, 255), (pg_pos[0], pg_pos[1]), 3, 1)
    pg_mean=to_pygame(vs.mean)
    pg.draw.circle(screen, (0, 0, 0), pg_mean, 3, 0)

def set_data(time):

    font = pg.font.Font(None, 36)
    pg_time = font.render("%.1f" % time, 1, (10, 10, 10))
    time_taken = font.render("time taken: ", 1, (10, 10, 10))
    #speed = font.render("%.3f" % vel, 1, (10, 10, 10))
    #w = font.render("%.3f" % omega, 1, (10, 10, 10))
    #omega = font.render("omega: ", 1, (10, 10, 10))
    #v = font.render("velocity: ", 1, (10, 10, 10))
    screen.blit(pg_time, (width / 10 * 6 + 10, 20))
    screen.blit(time_taken, (width / 10 * 5, 20))
    #screen.blit(w, (width / 10 * 9, 120))
    #screen.blit(omega, (width / 10 * 8, 120))
    #screen.blit(speed, (width / 10 * 9, 70))
    #screen.blit(v, (width / 10 * 8, 70))



def to_pygame(coords):
    '''Convert coordinates into pygame coordinates'''
    return (int(coords[0] * 5 + width / 2 - 150), int(coords[1] * -5 + height / 2 + 200))


# PyGame parameters
pg.init()
infoObject = pg.display.Info()
screen = pg.display.set_mode((infoObject.current_w, infoObject.current_h))
width = infoObject.current_w
height = infoObject.current_h
background_colour = (255, 255, 255)
screen.fill(background_colour)

# Data from the JSON file
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

# Plot the bounding polygon
pg_bounding_polygon = []
for point in bounding_polygon:
    pg_bounding_polygon.append(to_pygame(point))

'''Initialization'''
vs=Virtual_structure(len(formation_positions))
vs.set_formation(formation_positions)
vs.init_pos(traj_pos[0],traj_theta[0])
robots=Robots(start_positions,start_positions.shape[0])

set_bg()
pg.display.flip()
start = False
done = False
init_pos=False
time_step=0
total_time=0

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
    for t in range(20):
        if not init_pos:
            if not np.isclose(robots.locations,vs.desired_pos).all():
                vs.set_des_xi(traj_pos[time_step],traj_theta[time_step])
                vs.update_structure(robots.locations)
                robots.move(vs)
            else:
                init_pos=True
                time_step+=1
                robots.start=True
        else:
            #vs.set_des_xi(traj_pos[time_step],traj_theta[time_step])
            vs.init_pos(traj_pos[time_step],traj_theta[time_step])
            vs.update_structure(robots.locations)
            robots.move(vs)
            '''if time_step + 1 < len(traj_t):
                time_step+=1
                total_time+=1'''

            if (np.isclose(traj_pos[time_step], vs.mean,1e-2,1e-3).all()):
                if time_step+1<len(traj_t):
                    time_step += 1

            if time_step + 1 < len(traj_t):
                total_time+=1


    set_bg()
    set_data(total_time*0.1)
    pg.display.flip()


