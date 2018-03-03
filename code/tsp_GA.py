import numpy as np
import random
import matplotlib.pyplot as plt


class TSP_GA():

    def __init__(self, start, goal, nodes_dict, population_size, N):
        self.start = start
        self.goal = goal
        self.nodes = nodes_dict # assumed to come as a dictionary
        self.population_size = population_size
        self.N = N
        self.population = self.init_population()

    def genetic_algorithm(self, iterations=50, plot = False, epsilon = 0.01):
        """Takes a population with k individuals and a fitness function which evaluates
        the fitness of an individual."""

        for iteration in range(iterations):
            new_population = np.zeros(self.population.shape)
            for j in range(self.population_size):
                x = self.individual_selection(self.population)
                y = self.individual_selection(self.population)
                child = self.reproduce(x, y)
                if np.random.rand() < epsilon:
                    self.mutate(child)
                new_population[j] = child
            if plot and iteration%5 == 0:
                best_ind = self.best_individual(new_population)
                self.plot_individual(best_ind)
            self.population = new_population

        return self.best_individual(self.population)

    def fitness(self, individual):
        """A fitness function."""

        index = individual[0]
        cost = self.distance(start, nodes[index])
        previous_index = index
        for index in individual[1:]:
            cost += self.distance(nodes[previous_index], nodes[index])
            previous_index = index
        return -cost

    def individual_selection(self, population, selection_rule='tournament', batch_size = 5):
        """Takes a population as input, outputs a random individual."""

        if selection_rule == 'random':
            rand = np.random.randint(0, self.population_size)
            return population[rand]

        elif selection_rule == 'tournament':
            # returns the best individual of a subset of the input population
            batch_size = min(self.population_size, batch_size)
            batch_index = random.sample(range(self.population_size), batch_size)
            population_batch = population[batch_index]
            return self.best_individual(population_batch)

        else:
            raise ValueError('Not a supported selection rule.')

    def best_individual(self, population):
        """Returns the individual with highest fitness in a population."""

        best_fitness = -np.inf
        for individual in population:
            fitness = self.fitness(individual)
            if fitness > best_fitness:
                best_fitness = fitness
                best_individual = individual

        return best_individual

    def reproduce(self, x, y):
        """Order 1 cross over reproduction."""
        #todo: Make sure it's working

        child = -np.ones(x.shape)
        cuts = random.sample(range(x.size), 2)
        c1 = np.min(cuts); c2 = np.max(cuts)
        chromosome = x[c1:c2]
        child[c1:c2] = chromosome

        for elem in y:
            if elem not in chromosome:
                i = np.where(child == -1)[0][0]
                child[i] = elem

        return child


    def distance(self, p1, p2, measure='euclidean'):
        """Distance between points p1 and p2 with respect to some metric."""

        if measure == 'euclidean':
            return np.linalg.norm(p1 - p2)

        elif measure == 'visibility graph':
            pass

        else:
            raise ValueError('Not a supported measure.')

    def mutate(self, individual):
        """Mutate an individual, swap two of the 'nodes'."""

        i, j = random.sample(range(self.N), 2)
        individual[[i, j]] = individual[[j, i]]

    def init_population(self):
        """Returns a population, used for initialization."""
        return np.array([self.sample_individual() for i in range(self.population_size)])

    def sample_individual(self):
        individual = np.arange(self.N)
        np.random.shuffle(individual)
        return individual

    def plot_individual(self, individual):

        self.plot_nodes()
        self.plot_paths(individual)
        plt.axis([-1, 20, -1, 20])
        plt.show()

    def plot_nodes(self):
        # Plot start and goal
        plt.scatter(start[0], start[1], marker='*', c='b', s=100)
        plt.scatter(goal[0], goal[1], marker='*', c='r', s=100)

        # Plot other nodes
        for p in nodes.values():
            plt.scatter(p[0], p[1], marker='x', c='g', s=100)

    def plot_paths(self, individual):
        index = individual[0]
        plt.plot([start[0], nodes[index][0]], [start[1], nodes[index][1]], c='b')

        previous_index = index
        for index in individual:
            plt.plot([nodes[previous_index][0], nodes[index][0]],
                     [nodes[previous_index][1], nodes[index][1]], c='b')
            previous_index = index

        index = individual[-1]
        plt.plot([nodes[index][0], goal[0]], [nodes[index][1], goal[1]], c='b')


def create_nodes(N=5, plot=False):
    """Returns a dictionary with nodes, where the keys are their index.
    Just a help function to test the genetic algorithm."""

    # randomize N locations in region [0, 5) x [0, 5) and put in a dictionary
    locations = np.random.rand(N, 2) * 20
    nodes = {index: location for (index, location) in enumerate(locations)}

    if plot:
        for p in nodes.values():
            plt.scatter(p[0], p[1])
        plt.axis([-1, 20, -1, 20])
        plt.show()

    return nodes


population_size = 100; N = 15
start = np.array((0, 0))
goal = np.array((3, 0))

np.random.seed(0) # random seed 0, N = 50 gives best path 36.028
nodes = create_nodes(N)

# Initialize the GA
tsp_ga = TSP_GA(start, goal, nodes, population_size, N)

# show the best initialized individual in the population
best_init_ind = tsp_ga.best_individual(tsp_ga.population)
print('cost of best init. path: ', -tsp_ga.fitness(best_init_ind))
tsp_ga.plot_individual(best_init_ind)

# Run the algorithm and show the best after some iterations
best_ind = tsp_ga.genetic_algorithm(iterations=50, plot=True)
print('cost of GA path: ', -tsp_ga.fitness(best_ind))
tsp_ga.plot_individual(best_ind)
