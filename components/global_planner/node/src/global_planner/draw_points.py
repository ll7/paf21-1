"""Script to draw points in the Carla UI"""
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)


def draw_waypoint(location, num, life_time=50.0):
    """Function to draw a waypoint in the Carla UI"""
    world.debug.draw_string(location, str(num), draw_shadow=False,
                            color=carla.Color(r=0, g=0, b=0),
                            life_time=life_time, persistent_lines=True)


def draw_first_waypoint(location, life_time=50.0):
    """Function to draw the first waypoints in the Carla UI"""
    world.debug.draw_string(location, 'O', draw_shadow=False,
                            color=carla.Color(r=255, g=0, b=255),
                            life_time=life_time, persistent_lines=True)


def draw_last_waypoint(location, life_time=50.0):
    """Function to draw the last waypoints in the Carla UI"""
    world.debug.draw_string(location, 'O', draw_shadow=False,
                            color=carla.Color(r=0, g=0, b=255),
                            life_time=life_time, persistent_lines=True)


# def draw_nodes(location, life_time=50.0):
#     world.debug.draw_string(location, 'O', draw_shadow=False,
#                             color=carla.Color(r=255, g=0, b=0),
#                             life_time=life_time, persistent_lines=True)


# world = client.load_world('Town01')
world = client.get_world()
list_waypoints = [{'x': 327.9275207519531, 'y': -197.1593017578125},
                  {'x': 334.1826477050781, 'y': -194.5936737060547},
                  {'x': 336.7861328125, 'y': -188.3739013671875},
                  {'x': 336.78778076171875, 'y': -185.64996337890625},
                  {'x': 327.9275207519531, 'y': -197.1593017578125},
                  {'x': 334.1826477050781, 'y': -194.5936737060547},
                  {'x': 336.7861328125, 'y': -188.3739013671875},
                  {'x': 336.78778076171875, 'y': -185.64996337890625},
                  {'x': 336.8131408691406, 'y': -143.6699676513672},
                  {'x': 336.8131408691406, 'y': -143.6699676513672},
                  {'x': 336.82708740234375, 'y': -120.5799789428711},
                  {'x': 336.82708740234375, 'y': -120.5799789428711},
                  {'x': 336.85821533203125, 'y': -69.02999114990234},
                  {'x': 336.85821533203125, 'y': -69.02999114990234},
                  {'x': 336.85858154296875, 'y': -68.41999053955078},
                  {'x': 336.8603515625, 'y': -65.49959564208984},
                  {'x': 334.3076477050781, 'y': -59.76737594604492},
                  {'x': 328.3174133300781, 'y': -57.471012115478516},
                  {'x': 325.6300048828125, 'y': -57.471336364746094},
                  {'x': 325.1600036621094, 'y': -57.47139358520508},
                  {'x': 336.85858154296875, 'y': -68.41999053955078},
                  {'x': 336.8603515625, 'y': -65.49959564208984},
                  {'x': 334.3076477050781, 'y': -59.76737594604492},
                  {'x': 328.3174133300781, 'y': -57.471012115478516},
                  {'x': 325.6300048828125, 'y': -57.471336364746094},
                  {'x': 325.1600036621094, 'y': -57.47139358520508},
                  {'x': 167.1699981689453, 'y': -57.49064636230469},
                  {'x': 167.1699981689453, 'y': -57.49064636230469},
                  {'x': 144.989990234375, 'y': -57.493350982666016},
                  {'x': 144.989990234375, 'y': -57.493350982666016},
                  {'x': 144.99, 'y': -57.5}]

LIFE_TIME = 30
Z_AXIS = 1.0

for index, point in enumerate(list_waypoints):
    carla_point = carla.Location(x=point['x'], y=-point['y'], z=Z_AXIS)
    draw_waypoint(carla_point, index, LIFE_TIME)

first_point = list_waypoints[0]
last_point = list_waypoints[-1]

carla_point = carla.Location(x=first_point['x'], y=-first_point['y'], z=Z_AXIS)
draw_first_waypoint(carla_point, LIFE_TIME)

carla_point = carla.Location(x=last_point['x'], y=-last_point['y'], z=Z_AXIS)
draw_last_waypoint(carla_point, LIFE_TIME)

# vehicle_blueprint = client.get_world().get_blueprint_library().filter('model3')[0]
# spawn_point = random.choice(spawnpoints)
# vehicle = client.get_world().spawn_actor(vehicle_blueprint, spawn_point)
# vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))
# vehicle.destroy()
