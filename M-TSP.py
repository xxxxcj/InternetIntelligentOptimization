import array
import random
import json

import numpy

from deap import algorithms
from deap import base
from deap import creator
from deap import tools

# parameter
from matplotlib import pyplot as plt

from Map import Map

from color_names import color_names

POPULATION = 1000
MATE_PROBABILITY = 0.7
MUTATE_PROBABILITY = 0.2
GENERATION = 1000
TRAVELLER_NUM = 10

INDEPENDENT_PROBABILITY = 0.05  # 突变中每个位置的独立概率
TOURNSIZE = 3  # 锦标赛每轮人数

IND_SIZE = 100
MAX_POS_RANGE = int(1e3)

random.seed(166)

# gr*.json contains the distance map in list of list style in JSON format
# Optimal solutions are : gr17 = 2085, gr24 = 1272, gr120 = 6942
# with open("tsp/gr120.json", "r") as tsp_data:
#     tsp = json.load(tsp_data)

# distance_map = tsp["DistanceMatrix"]
# IND_SIZE = tsp["TourSize"]

my_map = Map(node_num=IND_SIZE, edge_num=int(0), max_pos_range=MAX_POS_RANGE, value_range=[0, 10])

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", array.array, typecode='i', fitness=creator.FitnessMin)

toolbox = base.Toolbox()

# Attribute generator
toolbox.register("indices", random.sample, [i for i in range(IND_SIZE + TRAVELLER_NUM)],
                 IND_SIZE + TRAVELLER_NUM)

# Structure initializers
toolbox.register("individual", tools.initIterate, creator.Individual, toolbox.indices)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)


def evalMTSP(individual, map: Map):
    distance = map.get_distance(0, IND_SIZE - 1)
    for gene1, gene2 in zip(individual[0:-1], individual[1:]):
        distance += map.get_distance(gene1, gene2)
    return distance,


toolbox.register("mate", tools.cxOrdered)
toolbox.register("mutate", tools.mutShuffleIndexes, indpb=INDEPENDENT_PROBABILITY)
toolbox.register("select", tools.selTournament, tournsize=TOURNSIZE)
toolbox.register("evaluate", evalMTSP, map=my_map)


def main():
    # random.seed(169)

    pop = toolbox.population(n=POPULATION)

    hof = tools.HallOfFame(1)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", numpy.mean)
    stats.register("std", numpy.std)
    stats.register("min", numpy.min)
    stats.register("max", numpy.max)

    pop, logbook = algorithms.eaSimple(pop, toolbox, MATE_PROBABILITY, MUTATE_PROBABILITY, GENERATION, stats=stats,
                                       halloffame=hof)

    return pop, stats, hof


if __name__ == "__main__":
    pop, stats, hof = main()

    points_x = []
    points_y = []
    for point in my_map.map_points:
        points_x.append(point.position[0])
        points_y.append(point.position[1])

    plt.figure()
    plt.plot(points_x, points_y, 'o')

    color_idx = random.randint(0, len(color_names))

    for city1, city2 in zip(hof[0][0:-1], hof[0][1:]):

        if city1 >= IND_SIZE:
            color_idx = random.randint(0, len(color_names))
            continue
        else:
            pos1 = my_map.map_points[city1].position

        if city2 >= IND_SIZE:
            color_idx = random.randint(0, len(color_names))
            continue
        else:
            pos2 = my_map.map_points[city2].position

        plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], linewidth=1,
                 color=color_names[color_idx])

    plt.show()
