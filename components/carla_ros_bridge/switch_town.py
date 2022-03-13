import sys
import carla


def switch_town(carla_host: str, town: str):
    client = carla.Client(carla_host, 2000)
    client.set_timeout(10.0)
    client.load_world(town)


def main():
    if len(sys.argv) != 3:
        raise ValueError('Invalid arguments! Expected CARLA host and town!')

    carla_host = sys.argv[1]
    town = sys.argv[2]
    switch_town(carla_host, town)


if __name__ == '__main__':
    main()
