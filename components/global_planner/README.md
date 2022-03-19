
# Global Planner

## About
The Global Planner processes XODR map data to serve navigation tasks as a ROS service.
Our navigation service uses a custom request message [NavigationRequest](./node/nav_srvs/srv/NavigationRequest.srv).
It can be queried like this [example](../local_planner/node/src/local_planner/navigation.py).

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
This technique draws a linear between the start / end point of a given geometry and puts some points
inbetween them. The interpolation conforms to a given distance between the generated points.

2) Circular Arc Interpolation
For curved geometries defined as circular arcs, we compute the circle from the curvature coefficient
and interpolate some uniformly distributed points on top of the circle between start / end position.

3) Centripetal Catmull–Rom Spline
For smoothing the trajectory, we additionally apply a centripetal spline to retrieve a more
streamlined route, facilitating dynamic driving. (There's a parameter *alpha* which tunes the dynamics)

### Route Annotation
After computing the route waypoints, we simulate driving the route and annotate useful metadata
such as:

- selected lane + possible lanes
- legal speed limit
- distance to next traffic light / end of lane

## Build Node + Run Tests

```sh
docker build . -t "global_planner"
```

## References
See our [wiki](https://github.com/ll7/paf21-1/wiki/Global-Planner) more detailed information.
