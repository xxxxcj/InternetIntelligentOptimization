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

# random.seed(166)

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
toolbox.register("indices", random.sample, [i for i in range(IND_SIZE + TRAVELLER_NUM-1)], IND_SIZE + TRAVELLER_NUM-1)

# Structure initializers
toolbox.register("individual", tools.initIterate, creator.Individual, toolbox.indices)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)


def evalMTSP(individual, map: Map):
    travelled_cities = []
    distance = 0
    for city in individual:
        if city >= IND_SIZE:
            if len(travelled_cities) > 1:
                traveller_distance = map.get_distance(travelled_cities[0], travelled_cities[-1])
                for gene1, gene2 in zip(travelled_cities[0:-1], travelled_cities[1:]):
                    traveller_distance += map.get_distance(gene1, gene2)
                distance += pow(traveller_distance, 1.1)
            travelled_cities.clear()
        else:
            travelled_cities.append(city)

    if len(travelled_cities) > 1:
        traveller_distance = map.get_distance(travelled_cities[0], travelled_cities[-1])
        for gene1, gene2 in zip(travelled_cities[0:-1], travelled_cities[1:]):
            traveller_distance += map.get_distance(gene1, gene2)
        distance += pow(traveller_distance, 1.1)

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

    color_idx = random.randint(0, len(color_names) - 1)

    print(hof[0])

    start_city = hof[0][0]
    for city1, city2 in zip(hof[0][0:-1], hof[0][1:]):
        if city1 >= IND_SIZE:
            color_idx = random.randint(0, len(color_names) - 1)
            start_city = city2
            continue
        else:
            pos1 = my_map.map_points[city1].position

        if city2 >= IND_SIZE:
            pos2 = my_map.map_points[start_city].position
        else:
            pos2 = my_map.map_points[city2].position

        plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], linewidth=1, color=color_names[color_idx])
    pos1 = my_map.map_points[start_city].position
    pos2 = my_map.map_points[hof[0][-1]].position
    plt.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], linewidth=1, color=color_names[color_idx])
    plt.show()
