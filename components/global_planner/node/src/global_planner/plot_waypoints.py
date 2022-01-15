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
    waypoints = [(396.6376037597656, -208.82986450195312), (394.3800048827421, -9.84999948643781),
                 (392.3810650632555, -8.970101400465818), (390.3990324152924, -1.272024735923396),
                 (385.62136837947946, 1.9794512510940647), (384.588933864571, 1.9799997182346263),
                 (384.588933864571, 1.9799997182346263), (348.2289389951926, 1.9993154831852014),
                 (347.6289390798563, 1.9996342251810926), (344.99203060323623, 2.0010350478300962),
                 (340.66658585563545, -0.7868873130541685), (338.89521732044915, -7.7862819772881),
                 (338.89340377248055, -10.788789990691912), (338.8718828632788, -46.41878349125918),
                 (338.8706181325178, -48.51267034204985), (335.633434179946, -53.43156030653416),
                 (327.86331602225545, -55.47106569600992), (325.6302379919994, -55.47133780278876),
                 (167.1702391684145, -55.490646595080754), (167.16999546284472, -57.49064658023266),
                 (144.99, -57.5)]
    plot_waypoints(waypoints)
    waypoints = [{'x': wp[0], 'y': wp[1]} for wp in waypoints]
    print(waypoints)
