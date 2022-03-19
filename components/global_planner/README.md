
# Global Planner

## About
The Global Planner processes XODR map data to serve navigation tasks.

### XODR Parser
The map is represented as XODR files matching the
[ASAM OpenDRIVE](https://www.asam.net/standards/detail/opendrive/) standard.

Roads are represented as lanelets interconnected by junctions. Each lanelet consists of
one or more geometries, each given as a start point, heading, length and arc curvature (optional).

Moreover, there is meta-information on traffic lights and speeds signs which we also evaluate
for enhanced route planning by useful metadata annotations.

### Shortest Paths
For computing shortest paths, we use Dijkstra's algorithm in a very simple form.
It's not particularly optimized for performance as the routes are generated on startup (even
before the car starts driving).

The navigation graph's edge weights are set to each lanelet's length which might be enhanced
by a more sophisticated logic considering traffic hotspots / wait times at traffic lights, etc.

### Route Interpolation
For retrieving the route points between our lanelets, we use several interpolation techniques such as:

1) Linear Interpolation
Draws a linear between the start / end point of a given geometry and puts some points inbetween
them with a given offset between the interpolated points.

2) Circular Arc Interpolation


3) Centripetal Catmull–Rom spline

Wiki: [Global-Planner](https://github.com/ll7/paf21-1/wiki/Global-Planner)

## Build Node + Run Tests

```sh
docker build . -t "global_planner"
```
