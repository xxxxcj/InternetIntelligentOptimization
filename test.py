from Map import *
import matplotlib.pyplot as plt

my_map = Map(int(1e2), int(0), int(1e3), [0, 10])

points_x = []
points_y = []
for point in my_map.map_points:
    points_x.append(point.position[0])
    points_y.append(point.position[1])

plt.figure()
plt.plot(points_x, points_y, 'o')
for point in my_map.map_points:
    points_x = []
    points_y = []
    for next_point in point.next:
        points_x.append(next_point.position[0])
        points_y.append(next_point.position[1])
    plt.plot(points_x, points_y, linewidth=1, color='red')
plt.show()
