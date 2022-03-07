"""Represents a kalman filter and is a wrapper for the opencv kalman filter."""

from typing import List, Tuple
import numpy as np
from cv2 import cv2


class KalmanFilter:
    """Represents a kalman filter and is a wrapper for the opencv kalman filter."""
    def __init__(self):
        self.kalman_filter = cv2.KalmanFilter(4, 2)
        self.kalman_filter.measurementMatrix = np.array([[1, 0, 0, 0],
                                                         [0, 1, 0, 0]], np.float32)
        self.kalman_filter.transitionMatrix = np.array([[1, 0, 1, 0],
                                                        [0, 1, 0, 1],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]], np.float32)
        self.kalman_filter.processNoiseCov = np.array([[1, 0, 0, 0],
                                                       [0, 1, 0, 0],
                                                       [0, 0, 1, 0],
                                                       [0, 0, 0, 1]], np.float32)
        self.last_prediction: Tuple[float, float] = (0.0, 0.0)

    def correct(self, position: Tuple[float, float]):
        """This function correct the position prediction of the object. """
        measured = np.array(position, dtype=np.float32)
        measured = measured[:, np.newaxis]
        self.kalman_filter.correct(measured)
        self.last_prediction = self._predict()

    def predict_points(self, num_points: int) -> List[Tuple[float, float]]:
        """This function estimates the position of the object. """
        first_pred = self.last_prediction
        predictions = [self._predict() for i in range(num_points)]
        predictions.insert(0, first_pred)
        return predictions

    def _predict(self) -> Tuple[float, float]:
        prediction = self.kalman_filter.predict()
        return prediction[0][0], prediction[1][0]
