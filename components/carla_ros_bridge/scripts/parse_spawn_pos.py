import os
import yaml
import tf

config_file = os.environ['CONFIG_FILE']
with open(config_file, encoding='utf-8') as file:
    config = yaml.safe_load(file)
    pos = config['competition']['start']['position']
    orient = config['competition']['start']['orientation']
    angles = tf.transformations.euler_from_quaternion(
        orient["x"], orient["y"], orient["z"], orient["w"])
    
    print(f'{pos["x"]}, {pos["y"]}, {pos["z"]}, {angles[0]}, {angles[1]}, {angles[2]}')



