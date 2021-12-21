import math
import random


class MapPoint:
    def __init__(self, index, value, max_pos_range):
        self.index = index
        self.value = value
        self.next = list()
        self.position = (random.randint(0, max_pos_range), random.randint(0, max_pos_range))

    def connect(self, next_point: "MapPoint"):
        self.next.append(next_point)
        next_point.next.append(self)

    def is_connected(self, point: "MapPoint"):
        return point in self.next

    def get_pos(self):
        return self.position

    def __repr__(self):
        return f"MapPoint: {self.index}, Value: {self.value}"


class Map:
    def __init__(self, node_num: int, edge_num: int, max_pos_range: int, value_range: list):
        self.map_points = [MapPoint(idx, random.randint(value_range[0], value_range[1]), max_pos_range) for idx in range(node_num)]

        connected_edge_num = 0
        while connected_edge_num < edge_num:
            a = random.randint(0, node_num - 1)
            b = random.randint(0, node_num - 1)
            if not self.map_points[a].is_connected(self.map_points[b]):
                self.map_points[a].connect(self.map_points[b])
                connected_edge_num += 1

    def get_distance(self, city1:int, city2:int):
        if city1 >= len(self.map_points) or city2 >= len(self.map_points):
            return 0
        return math.sqrt(pow(self.map_points[city1].position[0] - self.map_points[city2].position[0], 2) +
                         pow(self.map_points[city1].position[1] - self.map_points[city2].position[1], 2))
