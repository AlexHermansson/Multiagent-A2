import numpy as np
import json
import pyvisgraph as vg
import pygame as pg
import matplotlib.pyplot as plt
import time
import random

#np.random.seed(100)


class VRP_GA():

    def __init__(self, N, k, D_pg, D_pp, D_sp, D_sg, population_size):
        self.N = N; self.k = k
        self.D_pg = D_pg; self.D_pp = D_pp
        self.D_sp = D_sp; self.D_sg = D_sg
        self.population_size = population_size
        self.population, self.fitness_values, self.best_score, self.best_gene = self.init_population()
        self.best_scores=np.array([self.best_score])
        self.generation_scores=np.array(([self.best_score]))


    def genetic_algorithm(self, generations=50, plot = False, epsilon = 0.01):
        """Takes a population with k genes and a fitness function which evaluates
        the fitness of an gene. epsilon is the mutation rate"""

        for generation in range(generations):

            self.population = self.new_population(epsilon)
            self.fitness_values, best_score, best_gene = self.population_fitness(self.population)
            if best_score < self.best_score:
                self.best_score = best_score
                self.best_gene = best_gene
            if plot:
                self.best_scores=np.append(self.best_scores,self.best_score)
                self.generation_scores=np.append(self.generation_scores,best_score)


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

        for path in travel_list:
            L = len(path)
            cost = 0
            for i in range(1, L):

                if i == 1:
                    # if only start and goal in the path
                    if i == L - 1:
                        cost += D_sg[path[i - 1], path[i] - (self.k + self.N)]
                    else:
                        cost += D_sp[path[i - 1]][path[i] - self.k]

                else:

                    if i == L - 1:
                        cost += D_pg[path[i] - (self.k + self.N), path[i - 1] - self.k]
                    else:
                        cost += D_pp[path[i - 1] - self.k, path[i] - self.k]

            if cost > max_cost:
                max_cost = cost

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

            # if a goal
            elif elem >= self.k + self.N:
                goal_list.append(elem)

            # if a pickup point
            else:
                if not first_start:
                    pos_list = np.append(pos_list, elem)
                else:
                    path = np.append(path, elem)

        if pos_list.size > 0:
            path = np.append(path, pos_list)
        travel_list.append(path)

        for i, goal in enumerate(goal_list):
            travel_list[i] = np.append(travel_list[i], goal)

        return travel_list



    def gene_selection(self, selection_rule='tournament'):
        """Takes a population as input, outputs a random gene."""

        if selection_rule == 'random':
            rand = np.random.randint(0, self.population_size)
            return self.population[rand]

        elif selection_rule == 'tournament':
            # returns the best gene of a subset of the input population
            batch_size = int(self.population_size / 15)
            #batch_size = min(self.population_size, batch_size)
            batch_index = random.sample(range(self.population_size), batch_size)
            population_batch = self.population[batch_index]
            fitness_batch = self.fitness_values[batch_index]
            best_index = np.argmin(fitness_batch)
            return population_batch[best_index]

        else:
            raise ValueError('Not a supported selection rule.')

    def new_population(self,epsilon):
        new_population = np.zeros((self.population_size, self.N + (2 * self.k)),dtype=int)
        for j in range(self.population_size):
            x = self.gene_selection()  # parents x and y
            y = self.gene_selection()
            child1, child2 = self.crossover(x, y)
            if np.random.rand() < epsilon:
                self.mutate(child1)
            if np.random.rand() < epsilon:
                self.mutate(child2)
            new_population[j] = child1
            new_population[self.population_size - j - 1] = child2

        return new_population
    '''def best_gene(self, population):
        """Returns the gene with highest fitness in a population."""

        best_fitness = np.inf
        for gene in population:
            fitness = self.fitness(gene)
            if fitness < best_fitness:
                best_fitness = fitness
                best_gene = gene

        self.best_score = best_fitness
        return best_gene'''

    def crossover(self, x, y):
        """Order 1 cross over reproduction."""

        cuts = random.sample(range(x.size), 2)
        c1 = np.min(cuts); c2 = np.max(cuts)

        chromosome1 = x[c1:c2]
        child1 = -np.ones(x.shape,dtype=int)
        child1[c1:c2] = chromosome1

        for elem in y:
            if elem not in chromosome1:
                i = np.where(child1 == -1)[0][0]
                child1[i] = elem

        chromosome2 = y[c1:c2]
        child2 = -np.ones(x.shape,dtype=int)
        child2[c1:c2] = chromosome2

        for elem in x:
            if elem not in chromosome2:
                i = np.where(child2 == -1)[0][0]
                child2[i] = elem


        return child1, child2


    '''def distance(self, p1, p2, measure='euclidean'):
        """Distance between points p1 and p2 with respect to some metric."""

        if measure == 'euclidean':
            return np.linalg.norm(p1 - p2)

        elif measure == 'visibility graph':
            pass

        else:
            raise ValueError('Not a supported measure.')'''

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
        gene = np.arange(self.N + 2*self.k)
        np.random.shuffle(gene)
        return gene


'''def create_travel_list(gene, k, N):
    first_start = False  # says if we have found the first start position
    travel_list = []
    goal_list = []
    pos_list = np.array([],dtype=int)

    for i, elem in enumerate(gene):

        # if a start position
        if elem < k:
            if not first_start:
                path = np.array([elem])
                first_start = True
            else:
                travel_list.append(path)
                path = np.array([elem])

        # if a goal
        elif elem >= k + N:
            goal_list.append(elem)

        # if a pickup point
        else:
            if not first_start:
                pos_list = np.append(pos_list, elem)
            else:
                path = np.append(path, elem)

    if pos_list.size > 0:
        path = np.append(path, pos_list)
    travel_list.append(path)

    for i, goal in enumerate(goal_list):
        travel_list[i] = np.append(travel_list[i], goal)

    return travel_list


def fitness(gene, k, N):

    travel_list = create_travel_list(gene, k, N)
    max_cost = -np.inf

    for path in travel_list:
        L = len(path)
        cost = 0
        for i in range(1, L):

            if i == 1:
                # if only start and goal in the path
                if i == L-1:
                    cost += D_sg[path[i-1], path[i]-(k+N)]
                else:
                    cost += D_sp[path[i-1]][path[i]-k]

            else:

                if i == L-1:
                    cost += D_pg[path[i]-(k+N), path[i-1]-k]
                else:
                    cost += D_pp[path[i-1]-k, path[i]-k]

        if cost > max_cost:
            max_cost = cost

    return max_cost'''

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
    for edge in pg_edges:
        pg.draw.line(screen,(0,0,255),edge[0],edge[1])




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

points_of_interest=np.array(data["points_of_interest"])
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

for edge in g.visgraph.edges:
    p1=[edge.p1.x,edge.p1.y]
    p2=[edge.p2.x,edge.p2.y]
    pg_edges.append(list_to_pygame([p1,p2]))


#D_sp = set_distances(start_positions, points_of_interest, g)
#D_pp = point_distances(points_of_interest, g)
#D_pg = set_distances(goal_positions, points_of_interest, g)
#D_sg = set_distances(start_positions, goal_positions, g)
#np.save('D_sp', D_sp)
#np.save('D_pp', D_pp)
#np.save('D_pg', D_pg)
#np.save('D_sg', D_sg)

D_sp = np.load('D_sp.npy')
D_pp = np.load('D_pp.npy')
D_pg = np.load('D_pg.npy')
D_sg = np.load('D_sg.npy')

N = 10# number of pickup points
k = 3 # number of robots
pop_size = 50
generations = 50
#gene = np.arange(N + 2*k)
#np.random.shuffle(gene)
#test=fitness(gene,k,N)

vrp_ga = VRP_GA(N, k, D_pg, D_pp, D_sp, D_sg, pop_size)
vrp_ga.genetic_algorithm(generations,True)




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