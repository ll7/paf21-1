# pylint: disable=all

from typing import List, Tuple
from matplotlib import pyplot as plt


def plot_waypoints(points: List[Tuple[float, float]]):
    x_s = [-p[1] for p in points]
    y_s = [p[0] for p in points]
    labels = range(0, len(x_s))

    _, ax = plt.subplots()
    ax.scatter(x_s, y_s)

    for i, txt in enumerate(labels):
        ax.annotate(txt, (x_s[i], y_s[i]))

    plt.show()


# def print_roads():
#     _, ax = plt.subplots()
#     for road in xodr_map.lane_lets:
#         if road.road_id == 333:
#             points = [geo.start_point for geo in road.geometries]
#             xs = [-p[1] for p in points]
#             ys = [p[0] for p in points]
#             ax.scatter(xs, ys, c='red')
#             n = range(0, len(xs))
#             for i, txt in enumerate(n):
#                 ax.annotate(txt, (xs[i], ys[i]))
#         if road.road_id == 334:
#             points = [geo.start_point for geo in road.geometries]
#             xs = [-p[1] for p in points]
#             ys = [p[0] for p in points]
#             ax.scatter(xs, ys, c='green')
#             n = range(0, len(xs))
#             for i, txt in enumerate(n):
#                 ax.annotate(txt, (xs[i], ys[i]))
#         if road.road_id == 355:
#             points = [geo.start_point for geo in road.geometries]
#             points += [geo.end_point for geo in road.geometries]
#             xs = [-p[1] for p in points]
#             ys = [p[0] for p in points]
#             ax.scatter(xs, ys, c='blue')
#             n = range(0, len(xs))
#             for i, txt in enumerate(n):
#                 ax.annotate(txt, (xs[i], ys[i]))
#
#             points = [(150.99, -57.5)]
#             xs = [-p[1] for p in points]
#             ys = [p[0] for p in points]
#             ax.scatter(xs, ys, c='yellow')
#             n = range(0, len(xs))
#             for i, txt in enumerate(n):
#                 ax.annotate('P', (xs[i], ys[i]))
#     plt.savefig('test.png')
#     plt.show()


if __name__ == '__main__':
    waypoints = [(25.099697390777987, 172.2637269326465), (47.939327402889134, 172.13372343318662), (47.91940567573898, 168.63378013010478), (25.079775663627828, 168.76378362956467)]

    waypoints = [
         (384.5857464446122, -8.019998870703333), (348.2257515752338, -8.000683105752758), (348.22362662859456, -12.000682541327942), (384.58362149797296, -12.019998306278517),
         (384.5898369668927, -0.3199999572211037), (348.2298420975143, -0.3006841922705287), (348.22968272651633, -0.6006841499386673), (384.5896775958947, -0.6199999148892422),
         (384.5899963378906, -0.019999999552965164), (348.2300014685122, -0.0006842346023901782), (348.227876521873, -4.000683670177573), (384.5878713912514, -4.019999435128148),
         (384.5857464446122, -8.019998870703331), (348.2257515752338, -8.000683105752756), (348.227876521873, -4.000683670177573), (384.5878713912514, -4.019999435128148),
         (384.5895182248968, -0.9199998725573808), (348.2295233555184, -0.9006841076068058), (348.22968272651633, -0.6006841499386673), (384.5896775958947, -0.6199999148892422),
         (384.5814965513337, -16.0199977418537), (348.2215016819553, -16.000681976903124), (348.22362662859456, -12.000682541327942), (384.58362149797296, -12.019998306278517)]

    # waypoints = waypoints[250:]
    print(waypoints)
    plot_waypoints(waypoints)
