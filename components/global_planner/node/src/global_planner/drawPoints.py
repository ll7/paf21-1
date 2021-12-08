import carla
import random

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)


def draw_waypoint(location, life_time=50.0):
    world.debug.draw_string(location, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=0),
                            life_time=life_time, persistent_lines=True)


def draw_waypoint_first(location, life_time=50.0):
    world.debug.draw_string(location, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=255),
                            life_time=life_time, persistent_lines=True)


def draw_waypoint_last(location, life_time=50.0):
    world.debug.draw_string(location, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255),
                            life_time=life_time, persistent_lines=True)


def draw_nodes(location, life_time=50.0):
    world.debug.draw_string(location, 'O', draw_shadow=False, color=carla.Color(r=255, g=0, b=0),
                            life_time=life_time, persistent_lines=True)


# world = client.load_world('Town03')
world = client.get_world()
x = [151.0201317777701, 151.07926120004393, 151.10204015194478, 148.50182665159923, 147.4749138357087, 144.23591009984736, 143.5852898520519, 140.47595558397114, 138.70656948959413, 138.5366284522811, 138.62880268980723, 94.12424397700269, 90.35912903597337, 84.24546796806743, 81.6406182541614, 83.02911777156176, 83.02911777156176, 83.11314179063645, 83.11856761930079, 80.39256248632665, 73.33581213024362, 73.181824767485, 71.15196323802739, 70.80204352614126]
y = [62.77440875016973, 66.17389455120221, 67.4835086243901, 70.21767324165724, 70.25138800502465, 70.34722234490702, 70.36436269707242, 70.44627696969977, 70.49289079839156, 70.49736783278568, 73.99615389511308, 75.16861019644556, 75.26780077074915, 77.37459103506083, 82.78022497434345, 122.06733198799145, 122.06733198799145, 124.44476214748148, 124.59828406715253, 131.82042645729183, 134.95086402363816, 134.95416290284246, 134.99764873094657, 135.00514507895872]

list_waypoints = [{'x': 168.08001031270084, 'y': 0.04306259308466573}, {'x': 290.36387436468766, 'y': 0.03001132617077165}, {'x': 290.5761280623479, 'y': 0.029943620852621165}, {'x': 325.6300046575124, 'y': 0.01132171390924333}, {'x': 325.6303408028037, 'y': 0.011321535336526327}, {'x': 328.02516734033094, 'y': 0.010049315506384104}, {'x': 334.2096359709986, 'y': -2.2290054206649588}, {'x': 336.89511162507006, 'y': -7.9630831629196415}, {'x': 336.8934041373098, 'y': -10.789998012538765}, {'x': 336.8718832281081, 'y': -46.419991513106055}, {'x': 336.8715812226464, 'y': -46.91999142189874}, {'x': 336.87067520626124, 'y': -48.41999114827682}, {'x': 336.8649069019425, 'y': -57.96998940621726}, {'x': 336.85980300963956, 'y': -66.41998786481378}, {'x': 336.8585949877927, 'y': -68.41998749998453}, {'x': 336.8582265411294, 'y': -69.02998738871162}, {'x': 336.82708977802673, 'y': -120.57997798523826}, {'x': 336.81314316580483, 'y': -143.6699737732848}]

time2life = 30
z_axis = 1.0

for point in list_waypoints:
    carla_point = carla.Location(x=point['x'], y=-point['y'], z=z_axis)
    draw_waypoint(carla_point, time2life)

first_point = list_waypoints[0]
last_point = list_waypoints[-1]

carla_point = carla.Location(x=first_point['x'], y=-first_point['y'], z=z_axis)
draw_waypoint_first(carla_point, time2life)

carla_point = carla.Location(x=last_point['x'], y=-last_point['y'], z=z_axis)
draw_waypoint_last(carla_point, time2life)

'''''
vehicle_blueprint = client.get_world().get_blueprint_library().filter('model3')[0]
spawn_point = random.choice(spawnpoints)
vehicle = client.get_world().spawn_actor(vehicle_blueprint, spawn_point)
vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))
vehicle.destroy()

'''''
