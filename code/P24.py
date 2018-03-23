import triangle as tr
import json
import numpy as np

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
        obstacles.append(data[d])


face = tr.get_data('face')

np_bounding_polygon = np.array(bounding_polygon)
np_obstacles = 0
for obst in obstacles:
    np_obst = np.array(obst)


vertices = np.vstack((bounding_polygon, obstacles))
a = 0

#map_dict = {'vertices':}