import numpy as np
from cv2 import cv2
from local_planner.lane_detection import LaneDetection


def test_should_find_lines():
    assert(True)
    # orig_image = load_image('sample_001.png')
    # highl_image, lines = LaneDetection.highlight_lines(orig_image)
    # add some assertion

def load_image(file_name: str) -> np.ndarray:
    return cv2.imread('foobar', f'./test_resources/{file_name}')
