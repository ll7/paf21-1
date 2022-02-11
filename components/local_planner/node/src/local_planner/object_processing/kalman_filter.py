"""Represents a kalman filter and is a wrapper for the opencv kalman filter."""

from typing import Tuple
import numpy as np
from cv2 import cv2


class KalmanFilter:
    """Represents a kalman filter and is a wrapper for the opencv kalman filter."""
    # pylint: disable=too-few-public-methods
    def __init__(self, delta_time=0.1):
        self.kalman_filter = cv2.KalmanFilter(4, 2)
        # self.kalman_filter.measurementMatrix = np.array([[1, 0, 0, 0],
        #                                                  [0, 1, 0, 0]], np.float32)
        # self.kalman_filter.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1],
        #                                                 [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman_filter.measurementMatrix = np.array([[1, 0, 0, 0],
                                                         [0, 1, 0, 0]], np.float32)
        self.kalman_filter.transitionMatrix = np.array([[1, 0, delta_time, 0],
                                                        [0, 1, 0, delta_time],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]], np.float32)
        self.kalman_filter.processNoiseCov = np.array([[1, 0, 0, 0],
                                                       [0, 1, 0, 0],
                                                       [0, 0, 1, 0],
                                                       [0, 0, 0, 1]], np.float32) * 0.03
        self.last_prediction: Tuple[float, float] = (0.0, 0.0)

    def predict(self, position: Tuple[float, float]):
        """This function estimates the position of the object. """
        measured = np.array(position, dtype=np.float32)
        measured = measured[:, np.newaxis]
        self.kalman_filter.correct(measured)
        prediction = self.kalman_filter.predict()
        self.last_prediction = (prediction[0], prediction[1])
        return self.last_prediction
