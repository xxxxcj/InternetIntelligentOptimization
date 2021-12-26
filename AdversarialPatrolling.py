import random
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Circle
from matplotlib.axes import Axes

random.seed(15)


class Agent:
    def __init__(self, loc: int, status=0, vel=10):
        self.location = loc
        self.status = status
        self.Circle = Circle((-1, -1), 0.2, color='b')
        self.Circle.set_zorder(100)
        self.vel = vel


class Intruder:
    def __init__(self, loc: int, intruder_type: int):
        self.location = loc
        self.Circle = Circle((-1, -1), 0.2, color='yellow')
        self.Circle.set_zorder(100)
        self.type = intruder_type  # 0: 随机型小偷 1: 统计型小偷
        self.steal_time = 1.0
        self.start_time = -1

    def is_stealing(self):
        return self.start_time != -1

    def steal(self, t: float, map_point_history: list):
        # 返回值 True: 偷窃成功
        #       False: 正在偷窃

        if self.type == 0:
            if self.start_time == -1:
                self.start_time = t
                self.Circle.set_color("red")
            if t - self.start_time > self.steal_time:
                return True

        if self.type == 1:
            if self.start_time == -1:
                statistic = map_point_history[self.location]
                self.start_time = t
                self.Circle.set_color("red")
            if t - self.start_time > self.steal_time:
                return True

        return False


class Algorithm:
    def __init__(self):
        # 生成地图 也可以自己指定
        self.map_points: dict[int: list] = {
            i: (random.randint(0, 20), random.randint(0, 20)) for i in range(20)
        }
        print(self.map_points)

        # 生成连接边 也可以自己指定
        self.edges = [[random.randint(0, 19), random.randint(0, 19)] for _ in range(60)]

        # 生成巡逻智能体
        self.agents = [Agent(i) for i in range(5)]
        self.intruders: list[Intruder] = [Intruder(loc, random.randint(0, 1)) for loc in random.sample(range(len(self.map_points)), 5)]
        self.generate_intruder_gap = 7
        self.last_generate_time = 0

        # 记录偷窃相关数据
        self.random_intruder_steal_value = 0
        self.statistic_intruder_steal_value = 0

        # 计算距离矩阵
        self.dist_matrix = [[-1 for _ in self.map_points] for _ in self.map_points]
        self.compute_distance_matrix_and_delta_t()

        # 计算邻居节点
        self.neighbour_nodes = [[] for _ in self.map_points]
        self.compute_node_neighbour()

        self.average_idle_time = [0 for _ in self.map_points]
        self.k = [0 for _ in self.map_points]  # 被访问的次数
        self.arrive_time = [0 for _ in self.map_points]
        self.visited_history = [[] for _ in self.map_points]  # 每个节点的访问历史
        self.target_list = [-1 for _ in range(len(self.agents))]  # 统计目标地点，防止冲突

        # 绘图相关
        subplot = plt.subplots()
        self.fig = subplot[0]
        self.ax: Axes = subplot[1]
        plt.ion()
        # 绘制智能体初始位置
        for agent in self.agents:
            self.ax.add_artist(agent.Circle)
        # 绘制入侵者位置
        for intruder in self.intruders:
            intruder.Circle.center = self.map_points[intruder.location]
            self.ax.add_artist(intruder.Circle)

    def generate_intruder(self, t):
        if t - self.last_generate_time > self.generate_intruder_gap:
            idle_loc = list(range(len(self.map_points)))
            for i in self.intruders:
                idle_loc.remove(i.location)
            loc = random.sample(idle_loc, 1)[0]
            self.intruders.append(Intruder(loc, random.randint(0, 1)))
            self.intruders[-1].Circle.center = self.map_points[loc]
            self.ax.add_artist(self.intruders[-1].Circle)
            self.last_generate_time = t
            self.fig.canvas.draw()
            plt.pause(0.0001)

    def compute_distance_matrix_and_delta_t(self):
        dist_sum = 0
        for i in self.map_points:
            for j in self.map_points:
                if [i, j] in self.edges:
                    self.dist_matrix[j][i] = self.dist_matrix[i][j] = math.sqrt(
                        pow(self.map_points[i][0] - self.map_points[j][0], 2) + pow(
                            self.map_points[i][1] - self.map_points[j][1], 2))
                    dist_sum += self.dist_matrix[j][i]

        self.dist_average = dist_sum / len(self.edges)

    def compute_node_neighbour(self):
        for edge in self.edges:
            if edge[1] not in self.neighbour_nodes[edge[0]] and edge[1] != edge[0]:
                self.neighbour_nodes[edge[0]].append(edge[1])
            if edge[0] not in self.neighbour_nodes[edge[1]] and edge[1] != edge[0]:
                self.neighbour_nodes[edge[1]].append(edge[0])

        print(self.neighbour_nodes)

    def generate_map(self):
        points_x = []
        points_y = []
        for point in self.map_points:
            points_x.append(self.map_points[point][0])
            points_y.append(self.map_points[point][1])

        self.ax.plot(points_x, points_y, 'o')

        for edge in self.edges:
            self.ax.plot([self.map_points[edge[0]][0], self.map_points[edge[1]][0]],
                         [self.map_points[edge[0]][1], self.map_points[edge[1]][1]],
                         linewidth=1)

        plt.pause(0.01)

    def compute_location(self, start: int, target: int, distance: float):
        scale = distance / self.dist_matrix[start][target]
        x = self.map_points[target][0] - (self.map_points[target][0] - self.map_points[start][0]) * scale
        y = self.map_points[target][1] - (self.map_points[target][1] - self.map_points[start][1]) * scale
        return x, y

    def show_agent(self):
        for idx, target in enumerate(self.target_list):
            if target != -1:
                x, y = self.compute_location(self.agents[idx].location, target, self.agents[idx].status)
                self.agents[idx].Circle.center = x, y
            else:
                x, y = self.map_points[self.agents[idx].location]
                self.agents[idx].Circle.center = x, y
        self.fig.canvas.draw()
        plt.pause(0.0001)

    def get_distance(self, start: int, target: int):
        return self.dist_matrix[start][target]

    def update_average_idle_time(self, t, location):
        self.average_idle_time[location] = (self.k[location] * self.average_idle_time[location] + t - self.arrive_time[
            location]) / (self.k[location] + 1)

    def detect_intruder(self, agent: Agent):
        for intruder in self.intruders:
            if intruder.location == agent.location and intruder.is_stealing():
                intruder.Circle.remove()
                self.intruders.remove(intruder)

    def run_EGAI_random(self):
        self.generate_map()
        t = 0
        step_t = 0.1
        while True:
            self.generate_intruder(t)
            for intruder in self.intruders:
                if intruder.steal(t, self.visited_history):
                    intruder.Circle.remove()
                    if intruder.type == 0:
                        self.random_intruder_steal_value += 1
                    elif intruder.type == 0:
                        self.statistic_intruder_steal_value += 1
                    self.intruders.remove(intruder)
            # print(self.intruders)
            for agent_idx, agent in enumerate(self.agents):
                if agent.status == 0:
                    U = []
                    for neighbour in self.neighbour_nodes[agent.location]:
                        if neighbour in self.target_list:
                            continue
                        predict_t = t + self.dist_matrix[agent.location][neighbour] / agent.vel
                        predict_sum_of_average_idle_time = 0
                        instant_idle_time = predict_t - self.arrive_time[neighbour]
                        for i in range(len(self.average_idle_time)):
                            if i != neighbour:
                                predict_sum_of_average_idle_time += self.average_idle_time[i]
                            else:
                                predict_sum_of_average_idle_time += (self.k[i] * self.average_idle_time[
                                    i] + instant_idle_time) / (self.k[i] + 1)
                        predict_sum_of_average_idle_time /= len(self.average_idle_time)
                        # print(predict_sum_of_average_idle_time)
                        U.append([neighbour, instant_idle_time])

                    if len(U) != 0:
                        U.sort(key=lambda x: x[1], reverse=True)
                        sum_of_U = 0
                        for target, value in U:
                            sum_of_U += value

                        rand = random.random()
                        for target, value in U:
                            if rand > value / sum_of_U:
                                rand -= value / sum_of_U
                            else:
                                self.target_list[agent_idx] = target
                                agent.status = self.get_distance(agent.location, target)
                                break

                else:
                    if agent.status >= agent.vel * step_t:
                        agent.status -= agent.vel * step_t
                    else:
                        agent.status = 0
                        agent.location = self.target_list[agent_idx]

                        self.detect_intruder(agent)

                        self.update_average_idle_time(t, agent.location)
                        self.k[agent.location] += 1
                        self.arrive_time[agent.location] = t
                        self.visited_history[agent.location].append(t)
                        # print(self.average_idle_time)
                        # print(sum(self.average_idle_time) / len(self.average_idle_time))
                        self.target_list[agent_idx] = -1

            self.show_agent()

            t += step_t

    def run_EGAI(self):
        self.generate_map()
        t = 0
        step_t = 0.1
        while True:
            for agent_idx, agent in enumerate(self.agents):
                if agent.status == 0:
                    U = []

                    for neighbour in self.neighbour_nodes[agent.location]:
                        if neighbour in self.target_list:
                            continue
                        predict_t = t + self.dist_matrix[agent.location][neighbour] / agent.vel
                        predict_sum_of_average_idle_time = 0
                        instant_idle_time = predict_t - self.arrive_time[neighbour]
                        for i in range(len(self.average_idle_time)):
                            if i != neighbour:
                                predict_sum_of_average_idle_time += self.average_idle_time[i]
                            else:
                                predict_sum_of_average_idle_time += (self.k[i] * self.average_idle_time[
                                    i] + instant_idle_time) / (self.k[i] + 1)
                        predict_sum_of_average_idle_time /= len(self.average_idle_time)
                        # print(predict_sum_of_average_idle_time)
                        U.append([neighbour, instant_idle_time + predict_sum_of_average_idle_time])

                    if len(U) != 0:
                        U.sort(key=lambda x: x[1], reverse=True)

                        target = U[0][0]
                        self.target_list[agent_idx] = target
                        agent.status = self.get_distance(agent.location, target)

                else:
                    if agent.status >= agent.vel * step_t:
                        agent.status -= agent.vel * step_t
                    else:
                        agent.status = 0
                        agent.location = self.target_list[agent_idx]
                        self.update_average_idle_time(t, agent.location)
                        self.k[agent.location] += 1
                        self.arrive_time[agent.location] = t
                        self.visited_history[agent.location].append(t)
                        print(self.average_idle_time)
                        print(sum(self.average_idle_time) / len(self.average_idle_time))
                        self.target_list[agent_idx] = -1

            self.show_agent()

            t += step_t

    def run_randon_walk(self):
        self.generate_map()
        t = 0
        step_t = 0.1
        while True:
            for agent_idx, agent in enumerate(self.agents):
                if agent.status == 0:
                    U = []

                    for neighbour in self.neighbour_nodes[agent.location]:
                        if neighbour in self.target_list:
                            continue
                        predict_t = t + self.dist_matrix[agent.location][neighbour] / agent.vel
                        predict_sum_of_average_idle_time = 0
                        instant_idle_time = predict_t - self.arrive_time[neighbour]
                        for i in range(len(self.average_idle_time)):
                            if i != neighbour:
                                predict_sum_of_average_idle_time += self.average_idle_time[i]
                            else:
                                predict_sum_of_average_idle_time += (self.k[i] * self.average_idle_time[
                                    i] + instant_idle_time) / (self.k[i] + 1)
                        predict_sum_of_average_idle_time /= len(self.average_idle_time)
                        # print(predict_sum_of_average_idle_time)
                        U.append([neighbour, instant_idle_time])

                    if len(U) != 0:
                        target = random.sample(U, 1)[0][0]
                        self.target_list[agent_idx] = target
                        agent.status = self.get_distance(agent.location, target)

                else:
                    if agent.status >= agent.vel * step_t:
                        agent.status -= agent.vel * step_t
                    else:
                        agent.status = 0
                        agent.location = self.target_list[agent_idx]
                        self.update_average_idle_time(t, agent.location)
                        self.k[agent.location] += 1
                        self.arrive_time[agent.location] = t
                        self.visited_history[agent.location].append(t)
                        print(self.average_idle_time)
                        print(sum(self.average_idle_time) / len(self.average_idle_time))
                        self.target_list[agent_idx] = -1

            self.show_agent()

            t += step_t


a = Algorithm()
a.run_EGAI_random()
