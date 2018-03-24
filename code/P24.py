import triangle as tr
import triangle.plot as tr_plt
import json
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import *
import pyvisgraph as vg
import pygame as pg
import time
import random


class VRP_GA():

    def __init__(self, N, k, D_pg_, D_pp_, D_sp_, D_sg_, population_size,lambd,goal_positions):
        self.N = N; self.k = k
        self.D_pg = D_pg_; self.D_pp = D_pp_
        self.D_sp = D_sp_; self.D_sg = D_sg_
        self.population_size = population_size
        self.lambd = lambd
        self.goal_positions = goal_positions
        self.population, self.fitness_values, self.best_score, self.best_gene = self.init_population()
        self.best_scores=np.array([self.best_score])
        self.generation_scores=np.array(([self.best_score]))
        self.count=0

    def genetic_algorithm(self, generations=50, plot = False, epsilon = 0.01, percent_plot = True):
        """Takes a population with k genes and a fitness function which evaluates
        the fitness of an gene. epsilon is the mutation rate"""

        for generation in range(generations):

            self.population = self.new_population(epsilon)

            self.fitness_values, best_score, best_gene = self.population_fitness(self.population)
            if best_score < self.best_score:
                self.best_score = best_score
                self.best_gene = best_gene
            if self.generation_scores[-1]==best_score:
                self.count+=1
            else:
                self.count=0
            self.best_scores=np.append(self.best_scores,self.best_score)
            self.generation_scores=np.append(self.generation_scores,best_score)

            if self.count>10:
                self.check_unique()
                self.count=0


            if percent_plot:
                self.plot_percentage(generation, generations)

    def remove_best(self):
        indices=np.argsort(self.fitness_values)
        sorted_population=self.population[indices]
        for i in range(int(self.population_size*0.50)):
            sorted_population[i]=self.sample_gene()
        self.population=sorted_population


    def plot_percentage(self, generation, generations):
        """Function to plot the percentage of the iterations"""
        if (generation*100/generations)%10 == 0:
            print(int((generation/generations)*100), '%')

    def check_unique(self):
        '''we remove all the duplicates and we add random genes instead'''
        self.population=np.unique(self.population,axis=0)
        if self.population.shape[0]< self.population_size:
            random_population = np.array([self.sample_gene() for i in range(self.population_size-self.population.shape[0])])
            self.population=np.append(self.population,random_population,axis=0)


    def population_fitness(self, population):
        """Calculates the fitness of a population"""
        fitness_values = np.zeros(self.population_size)
        best_index = 0
        best_fitness = np.inf
        for i, gene in enumerate(population):
            fitness = self.fitness(gene)
            fitness_values[i] += fitness
            if fitness < best_fitness:
                best_index = i
                best_fitness = fitness

        best_gene = population[best_index]

        return fitness_values, best_fitness, best_gene

    def fitness(self, gene):

        travel_list = self.create_travel_list(gene)
        max_cost = -np.inf
        total_cost=0
        for path in travel_list:
            L = len(path)
            cost = 0
            for i in range(L):

                if i == 0:
                    # if only start and goal in the path
                    if i == L-1:
                        cost += D_sg[path[i], path[i]]
                    else:
                        cost += D_sp[path[i]][path[i+1] - self.k]

                else:

                    if i == L - 1:
                        cost += D_pg[path[0], path[i] - self.k]
                    else:
                        cost += D_pp[path[i] - self.k, path[i+1] - self.k]

            if cost > max_cost:
                max_cost = cost
            total_cost+=cost

        loss=total_cost+self.lambd*max_cost

        return max_cost#loss

    def create_travel_list(self, gene):
        first_start = False  # says if we have found the first start position
        travel_list = []
        goal_list = []
        pos_list = np.array([], dtype=int)

        for i, elem in enumerate(gene):

            # if a start position
            if elem < self.k:
                if not first_start:
                    path = np.array([elem])
                    first_start = True
                else:
                    travel_list.append(path)
                    path = np.array([elem])

            # if a pickup point
            else:
                if not first_start:
                    pos_list = np.append(pos_list, elem)
                else:
                    path = np.append(path, elem)

        if pos_list.size > 0: #if we had pickup before the first start point in the gene
            path = np.append(path, pos_list)
        travel_list.append(path)

        #for i in range(len(travel_list)):
            #travel_list[i] = np.append(travel_list[i], self.goal_positions[travel_list[i][0]])

        return travel_list



    def gene_selection(self, selection_rule='tournament',batch_size=None):
        """Takes a population as input, outputs a random gene."""

        if selection_rule == 'random':
            rand = np.random.randint(0, self.population_size)
            return self.population[rand]

        elif selection_rule == 'tournament':
            # returns the best gene of a subset of the input population
            if not batch_size:
                batch_size = int(self.population_size/100)
            batch_index = random.sample(range(self.population_size), batch_size)
            population_batch = self.population[batch_index]
            fitness_batch = self.fitness_values[batch_index]
            best_index = np.argmin(fitness_batch)
            return population_batch[best_index]

        else:
            raise ValueError('Not a supported selection rule.')

    def new_population(self, epsilon):
        new_population = np.zeros((self.population_size, self.N + self.k), dtype=int)
        for j in range(self.population_size):
            x = self.gene_selection(batch_size=4)  # parents x and y
            y = self.gene_selection(batch_size=4)
            child1, child2 = self.crossover(x, y)
            if np.random.rand() < epsilon:
                self.mutate(child1)
            if np.random.rand() < epsilon:
                self.mutate(child2)
            new_population[j] = child1
            new_population[self.population_size - j - 1] = child2

        return new_population

    def crossover(self, x, y):
        """Order 1 cross over reproduction."""

        cuts = random.sample(range(x.size), 2)
        c1 = np.min(cuts); c2 = np.max(cuts)

        chromosome1 = x[c1:c2]
        child1 = -np.ones(x.shape,dtype=int)
        child1[c1:c2] = chromosome1

        for elem in y:
            if elem not in chromosome1:
                #try:
                i = np.where(child1 == -1)[0][0]
                #except:
                a=0
                child1[i] = elem

        chromosome2 = y[c1:c2]
        child2 = -np.ones(x.shape,dtype=int)
        child2[c1:c2] = chromosome2

        for elem in x:
            if elem not in chromosome2:
                i = np.where(child2 == -1)[0][0]
                child2[i] = elem


        return child1, child2

    def crossover2(self, x, y):
        """Order 1 cross over reproduction."""

        cut = np.random.randint(0,x.size)

        chromosome1 = x[:cut]
        child1 = -np.ones(x.shape,dtype=int)
        child1[:cut] = chromosome1

        for elem in y:
            if elem not in chromosome1:
                i = np.where(child1 == -1)[0][0]
                child1[i] = elem

        chromosome2 = y[:cut]
        child2 = -np.ones(x.shape,dtype=int)
        child2[:cut] = chromosome2

        for elem in x:
            if elem not in chromosome2:
                i = np.where(child2 == -1)[0][0]
                child2[i] = elem

        if np.array_equal(child1,child2):
            self.mutate(child2)

        return child1, child2

    def mutate(self, gene):
        """Mutate an gene, swap two of the 'nodes'."""

        i, j = random.sample(range(self.N), 2)
        gene[[i, j]] = gene[[j, i]]

    def init_population(self):
        """Returns a population, used for initialization."""
        population = np.array([self.sample_gene() for i in range(self.population_size)])
        fitness_values, best_fitness, best_gene = self.population_fitness(population)
        return population, fitness_values, best_fitness, best_gene

    def sample_gene(self):
        gene = np.arange(self.N + self.k)
        np.random.shuffle(gene)
        return gene


def set_bg():
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    '''set obstacles'''
    [pg.draw.polygon(screen, (0, 0, 0), pg_obstacle, 0) for pg_obstacle in pg_obstacles]
    '''set boundaries'''
    pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)
    '''set start and goal positions'''
    for i,tl in enumerate(real_tl):
        pg.draw.circle(screen,cols[i],to_pygame(tl[0]),4)
        for j in range(1,len(tl)):
            if j==len(tl)-1:
                pg.draw.circle(screen,cols[i],to_pygame(tl[j]),4,1)
                pg.draw.line(screen,cols[i],to_pygame(tl[j-1]),to_pygame(tl[j]))
            else:
                pg.draw.circle(screen, cols[i], to_pygame(tl[j]), 2, 1)
                pg.draw.line(screen, cols[i],to_pygame(tl[j-1]),to_pygame(tl[j]))
    '''for edge in pg_edges:
        pg.draw.line(screen,(0,255,0),edge[0],edge[1])'''
    for point in points_to_visit:
        pg.draw.circle(screen,(0,0,0),to_pygame(point),4,1)


def list_to_pygame(list_of_points):
    pg_list_of_points=[]
    for point in list_of_points:
        pg_list_of_points.append(to_pygame(point))
    return  pg_list_of_points
def colors(n):
    #col=list(itertools.permutations([255,0,100,225],3))
    red=(255,0,0)
    orange=(255,100,0)
    green=(0,255,0)
    light_blue=(0,255,255)
    blue=(0,0,255)
    purple=(100,0,255)
    pink=(225,0,255)
    col=[red,orange,green,light_blue,blue,purple,pink]
    '''ret = []
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
        ret.append(random.choice(col))'''

    return col

def to_pygame(coords):
    '''Convert coordinates into pygame coordinates'''
    return (int(coords[0] * 14 + width / 2 - 150), int(coords[1] * -14 + height / 2 + 200))


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

    return best_cluster, triangles, vertices

def greedy_set_cover(triangles, sh_obstacles, vertices, sensor_range):

    points_to_visit = []
    cluster_list =[]
    while triangles:
        cluster, triangles, vertices = remove_best_cluster(triangles, sh_obstacles, vertices, sensor_range)
        points_to_visit.append(cluster.coords)
        cluster_list.append(cluster)

    return points_to_visit, cluster_list

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

def path_decoder(paths):
    travel_lists = []
    for path in paths:
        point_list = []
        for elem in path:
            if elem < k:
                point_list.append(start_positions[elem])
            elif elem >= k:
                point_list.append(points_to_visit[elem - k])

        point_list.append(goal_positions[path[0]])
        travel_lists.append(point_list)
    return travel_lists

def real_travel_list(travel_list):

    real_travel_list = []
    for list in travel_list:

        real_list = []
        for i in range(1, len(list)):
            p1 = vg.Point(list[i-1][0], list[i-1][1])
            p2 = vg.Point(list[i][0], list[i][1])
            shortest_path = g.shortest_path(p1, p2)
            for point in shortest_path:
                real_list.append(to_np(point))
        real_travel_list.append(real_list)

    return real_travel_list



'''set pygame env'''
pg.init()
infoObject = pg.display.Info()
screen = pg.display.set_mode((infoObject.current_w, infoObject.current_h))
width = infoObject.current_w
height = infoObject.current_h
background_colour = (255, 255, 255)
screen.fill(background_colour)


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

cols=colors(6)

'''set pygame obstacles anf bounding polygon'''
pg_obstacles=[]
for obstacle in obstacles:
    pg_obstacles.append(list_to_pygame(obstacle))

pg_bounding_polygon=list_to_pygame(bounding_polygon)

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

points_to_visit, clusters = greedy_set_cover(triangles, sh_obstacles, obst_vertices, sensor_range)
#triangles, vertices = remove_best_cluster(triangles, sh_obstacles, obstacles, sensor_range)

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
    #pg_edges.append(list_to_pygame([p1,p2]))

D_sp = set_distances(start_positions, points_to_visit, g)
D_pp = point_distances(points_to_visit, g)
D_pg = set_distances(goal_positions, points_to_visit, g)
D_sg = set_distances(start_positions, goal_positions, g)
np.save('D_sp_24', D_sp)
np.save('D_pp_24', D_pp)
np.save('D_pg_24', D_pg)
np.save('D_sg_24', D_sg)

#D_sp = np.load('D_sp_24.npy')
#D_pp = np.load('D_pp_24.npy')
#D_pg = np.load('D_pg_24.npy')
#D_sg = np.load('D_sg_24.npy')

N = len(points_to_visit) # number of pickup points
k = len(start_positions) # number of robots
pop_size = 500
generations = 20
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

best_gene=vrp_ga.best_gene
paths=vrp_ga.create_travel_list(best_gene)
travel_list = path_decoder(paths)
real_tl = real_travel_list(travel_list)



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

#todo: sample points