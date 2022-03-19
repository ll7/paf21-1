import os
import sys
import yaml
import tf
from math import degrees


def print_spawn_position(config):
    pos = config['competition']['start']['position']
    orient = config['competition']['start']['orientation']

    # convert the quaternion to euler angles (in degrees)
    angles = tf.transformations.euler_from_quaternion(
        [orient["x"], orient["y"], orient["z"], orient["w"]])
    angles = [degrees(a) for a in angles]

    # write the parsed spawn position in the format the
    # carla_example_ego_vehicle.launch file expects
    print(f'{pos["x"]}, {-pos["y"]}, {pos["z"]}, {angles[0]}, {angles[1]}, {angles[2]}')


def print_town(config):
    print(f"{config['competition']['town_name']}")


def print_num_cars(config):
    print(f"{config['competition']['spawn_npc']['n']}")


def print_num_pedestrians(config):
    print(f"{config['competition']['spawn_npc']['w']}")


def print_npc_spawn_seed(config):
    print(f"{config['competition']['spawn_npc']['s']}")


def main():
    config_file = sys.argv[1]
    part = sys.argv[2]

    with open(config_file, encoding='utf-8') as file:
        config = yaml.safe_load(file)

        if part == '--town':
            print_town(config)
        if part == '--spawn-pos':
            print_spawn_position(config)
        if part == '--num-cars':
            print_num_cars(config)
        if part == '--num-peds':
            print_num_pedestrians(config)
        if part == '--spawn-seed':
            print_npc_spawn_seed(config)


if __name__ == '__main__':
    main()
