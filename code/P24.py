import triangle as tr
import json
import numpy as np
import matplotlib.pyplot as plt

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

def to_vertices(bounding_polygon, obstacles):
    """A function to take all vertices of the bounding polygon and obstacles
    and put them into one large array."""
    vertices = np.array(bounding_polygon)
    for obst in obstacles:
        np_obst = np.array(obst)
        vertices = np.vstack((vertices, np_obst))

    return vertices


vertices = to_vertices(bounding_polygon, obstacles)
holes = polygon_holes(obstacles)
a = 0
plt.scatter(vertices[:,0], vertices[:,1])
plt.show()

map_dict = {'vertices':vertices, 'holes':holes}
t = tr.triangulate(map_dict, 'p')
a = 0