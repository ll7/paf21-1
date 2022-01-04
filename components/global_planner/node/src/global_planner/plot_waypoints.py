from typing import List, Tuple
from matplotlib import pyplot as plt

def plot_waypoints(waypoints: List[Tuple[float, float]]):
    xs =  [p[0] for p in waypoints]
    ys =  [p[1] for p in waypoints]
    n = range(0, len(xs))

    _, ax = plt.subplots()
    ax.scatter(xs, ys)

    for i, txt in enumerate(n):
        ax.annotate(txt, (xs[i], ys[i]))

    plt.show()
