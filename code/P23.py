import json
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import *
import pyvisgraph as vg
import pygame as pg
import time
import random
from numpy.linalg import norm


class Cluster():

    def __init__(self,coords):
        self.coords=coords
        self.neighbours=[]
        self.seen=False
        self.count=0

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
                #self.check_unique()
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

        loss=(1-self.lambd)*total_cost+self.lambd*max_cost

        return loss

    def fitness1(self, gene):

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

        return max_cost

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
            x = self.gene_selection(batch_size=6)  # parents x and y
            y = self.gene_selection(batch_size=6)
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


def set_bg(boolean):
    '''set initial and final position'''
    screen.fill((255, 255, 255))
    '''set obstacles'''
    [pg.draw.polygon(screen, (0, 0, 0), pg_obstacle, 0) for pg_obstacle in pg_obstacles]
    '''set boundaries'''
    pg.draw.polygon(screen, (0, 0, 0), pg_bounding_polygon, 1)
    '''set start and goal positions'''

    if boolean:

        for point in points_of_interest:
            pg.draw.circle(screen,(0,0,0),to_pygame(point),1)

        for cluster in start_goal_clusters:
            pg.draw.circle(screen,(255,0,0),to_pygame(cluster.coords),2)
            pg.draw.circle(screen, (0, 255, 0), to_pygame(cluster.coords), 14*sensor_range,1)
        for cluster in best_clusters:
            pg.draw.circle(screen, (0, 0, 255), to_pygame(cluster.coords), 2)
            pg.draw.circle(screen, (0, 255, 0), to_pygame(cluster.coords), 14 * sensor_range, 1)

    else:

        for i,tl in enumerate(real_tl):
            pg.draw.circle(screen,cols[i],to_pygame(tl[0]),4)
            for j in range(1,len(tl)):
                if j==len(tl)-1:
                    pg.draw.circle(screen,cols[i],to_pygame(tl[j]),4,1)
                    pg.draw.line(screen,cols[i],to_pygame(tl[j-1]),to_pygame(tl[j]))
                else:
                    pg.draw.circle(screen, cols[i], to_pygame(tl[j]), 2, 1)
                    pg.draw.line(screen, cols[i],to_pygame(tl[j-1]),to_pygame(tl[j]))


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
    return (int(coords[0] * 12 + width / 2 - 150), int(coords[1] * -12 + height / 2 + 200))

def remove_start_goal(start_points, goal_points, points, sh_obst, range):

    start_goal_list = []

    for start in start_points:
        index = 0
        cluster = Cluster(start)
        for point in points:

            if norm(start - point) < range:
                if visible(start, point, sh_obst):
                    points = np.delete(points, index, axis=0)
                    cluster.neighbours.append(Cluster(point))

                else:
                    index += 1
        start_goal_list.append(cluster)

    for goal in goal_points:
        index = 0
        cluster = Cluster(goal)
        for point in points:

            if norm(goal - point) < range:
                if visible(goal, point, sh_obst):
                    points = np.delete(points, index, axis=0)
                    cluster.neighbours.append(Cluster(point))
                else:
                    index += 1
        start_goal_list.append(cluster)

    return points, start_goal_list

def visible(point1, point2, sh_obst):
    '''Function to check if two points are visible to each other.'''

    line = LineString((Point(point1), Point(point2)))

    vis = True
    for obst in sh_obst:
        if line.intersects(obst):
            vis = False
            break
    if not sh_bounding_polygon.contains(line):
        vis=False

    return vis

def points_to_clusters(points):
    '''Create list of clusters from points on coordinate form.'''

    cluster_list = []
    for point in points:
        cluster_list.append(Cluster(point))

    return cluster_list

def assign_neighbours(clusters):
    '''Function to assign neighbour clusters to each point of interest'''

    for i,this_cluster in enumerate(clusters):
        for other_cluster in clusters[i+1:]:
            if norm(this_cluster.coords-other_cluster.coords)<sensor_range:
                if visible(this_cluster.coords, other_cluster.coords, sh_obstacles):
                    this_cluster.neighbours.append(other_cluster)
                    other_cluster.neighbours.append(this_cluster)
                    this_cluster.count+=1
                    other_cluster.count+=1

    return clusters

def remove_greedily(clusters):
    '''Remove the best cluster, i.e. greedily with respect to how many neighbours it has.'''

    best_cluster = clusters[0]
    best_count = 0
    for index, cluster in enumerate(clusters):

        if cluster.count > best_count:
            best_cluster = cluster
            best_count = cluster.count

    best_cluster.seen = True
    for neighbour in best_cluster.neighbours:
        neighbour.seen = True
        clusters.remove(neighbour)

    clusters.remove(best_cluster)
    clusters = update_neighbours(clusters)

    return best_cluster, clusters

def update_neighbours(clusters):
    '''Update neighbourhoods of remaining clusters'''

    for cluster in clusters:
        index = 0
        for neighbour in cluster.neighbours:
            if neighbour.seen:
                cluster.neighbours.pop(index)
            else:
                index += 1

        cluster.count = index

    return clusters

def find_clusters(clusters):
    '''Remove points from clusters greedily until no more are remaining.'''

    best_clusters = []
    while clusters:

        best_cluster, clusters = remove_greedily(clusters)
        best_clusters.append(best_cluster)

    return best_clusters

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

cols=colors(6)

'''Loading the data from json'''
data = json.load(open('P23_X.json'))

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

sh_bounding_polygon=Polygon(bounding_polygon)

'''set pygame obstacles anf bounding polygon'''
pg_obstacles=[]
for obstacle in obstacles:
    pg_obstacles.append(list_to_pygame(obstacle))

pg_bounding_polygon=list_to_pygame(bounding_polygon)

#np.random.shuffle(points_of_interest)

points_0, start_goal_clusters = remove_start_goal(start_positions, goal_positions,
                                                  points_of_interest, sh_obstacles, sensor_range)

cluster_list = points_to_clusters(points_0)
cluster_list = assign_neighbours(cluster_list)


best_clusters = find_clusters(cluster_list)

points_to_visit=[]
for cluster in best_clusters:
    points_to_visit.append(cluster.coords)


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

'''D_sp = set_distances(start_positions, points_to_visit, g)
np.save('D_sp_23_x', D_sp)
print('ciao')
D_pp = point_distances(points_to_visit, g)
np.save('D_pp_23_x', D_pp)
print('ciao')
D_pg = set_distances(goal_positions, points_to_visit, g)
np.save('D_pg_23_x', D_pg)
print('ciao')
D_sg = set_distances(start_positions, goal_positions, g)
np.save('D_sg_23_x', D_sg)
print('ciao')'''

D_sp = np.load('D_sp_23_x.npy')
D_pp = np.load('D_pp_23_x.npy')
D_pg = np.load('D_pg_23_x.npy')
D_sg = np.load('D_sg_23_x.npy')

N = len(points_to_visit) # number of pickup points
k = len(start_positions) # number of robots
pop_size = 2000
generations = 300
#n_trials=30
#N = 5# number of pickup points
#k = 3 # number of robots
lambd=0.7
vrp_ga = VRP_GA(N, k, D_pg, D_pp, D_sp, D_sg, pop_size,lambd,goal_positions)

vrp_ga.genetic_algorithm(generations,True, 0.01)
plt.plot(vrp_ga.best_scores)
plt.plot(vrp_ga.generation_scores)
plt.xlabel('epoch')
plt.ylabel('objective')
plt.show()
print(vrp_ga.best_score)


best_gene=vrp_ga.best_gene
#best_gene=np.loadtxt('p24xbest.txt',dtype=int)
print(vrp_ga.fitness1(best_gene)/vehicle_v_max)
paths=vrp_ga.create_travel_list(best_gene)
travel_list = path_decoder(paths)
real_tl = real_travel_list(travel_list)
np.savetxt('77sec_bestgene_23.txt',best_gene,fmt='%i')


time_step=0
start = False
done = False
show_points=True
i=0

while not done:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            done = True
    t0 = time.time()
    t1 = 0
    if not start:
        while (t1 - t0 < 1):
            t1 = time.time()
        start=True

    set_bg(show_points)
    pg.display.flip()
    t0 = time.time()
    t1 = 0
    while (t1 - t0 < 5):
        t1 = time.time()
    show_points = False
