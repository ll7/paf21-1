#!/bin/sh
(cd ../scenarios/default_scenario/; docker logs default_scenario_carla-ros-bridge_1 > ros_bridge_log.txt)
(cd ../scenarios/default_scenario/; docker logs default_scenario_perception_1 > perception_log.txt)
(cd ../scenarios/default_scenario/; docker logs default_scenario_local-planner_1 > local-planner_log.txt)
(cd ../scenarios/default_scenario/; docker logs default_scenario_global-planner_1 > global_log.txt)
(cd ../scenarios/default_scenario/; docker logs default_scenario_global-planner_1 > global_log.txt)
(cd ../scenarios/default_scenario/; docker-compose down)
