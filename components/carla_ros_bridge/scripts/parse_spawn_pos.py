import sys
import yaml
import tf
from math import degrees


def main():
    config_file = sys.argv[1]

    with open(config_file, encoding='utf-8') as file:
        config = yaml.safe_load(file)
        pos = config['competition']['start']['position']
        orient = config['competition']['start']['orientation']

        # convert the quaternion to euler angles (in degrees)
        angles = tf.transformations.euler_from_quaternion(
            [orient["x"], orient["y"], orient["z"], orient["w"]])
        angles = [degrees(a) for a in angles]

        # write the parsed spawn position to stdout in the format the
        # carla_example_ego_vehicle.launch file expects the spawn_point setting
        print(f'{pos["x"]}, {-pos["y"]}, {pos["z"]}, {angles[0]}, {angles[1]}, {angles[2]}')


if __name__ == '__main__':
    main()
