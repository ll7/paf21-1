"""Sensor fusion combining IMU and odometry data for improved vehicle state estimation.

Implements an Extended Kalman Filter (EKF) that fuses:
  - IMU angular velocity (gyroscope) and linear acceleration (accelerometer)
    as the prediction/motion model
  - Odometry position and velocity as correction measurements
  - IMU quaternion-derived orientation as an additional correction measurement

State vector: [x, y, theta, v]
  - x, y: 2D position in metres (Cartesian)
  - theta: heading angle in radians
  - v: forward speed in m/s
"""

from dataclasses import dataclass, field
from math import cos, sin
from typing import Tuple

import numpy as np

from vehicle_control.core.geometry import norm_angle


@dataclass
class SensorFusion:
    """Extended Kalman Filter fusing IMU and odometry for vehicle state estimation."""

    # ---- EKF state & covariance ----
    _state: np.ndarray = field(default_factory=lambda: np.zeros(4))
    _cov: np.ndarray = field(default_factory=lambda: np.eye(4))

    # ---- Noise matrices ----
    # Process noise Q – how much we trust the IMU motion model
    _proc_noise: np.ndarray = field(
        default_factory=lambda: np.diag([0.05, 0.05, 0.01, 0.5])
    )
    # Odometry measurement noise R_odom – [x, y, v]
    _odom_noise: np.ndarray = field(
        default_factory=lambda: np.diag([0.1, 0.1, 0.05])
    )
    # IMU orientation measurement noise R_imu – [theta]
    _imu_noise: np.ndarray = field(
        default_factory=lambda: np.array([[0.005]])
    )

    _initialized: bool = False

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def is_initialized(self) -> bool:
        """True once the filter has been seeded with an initial state."""
        return self._initialized

    @property
    def position(self) -> Tuple[float, float]:
        """Fused 2-D position estimate."""
        return float(self._state[0]), float(self._state[1])

    @property
    def velocity_mps(self) -> float:
        """Fused forward-speed estimate in m/s."""
        return float(self._state[3])

    @property
    def orientation_rad(self) -> float:
        """Fused heading-angle estimate in radians."""
        return float(self._state[2])

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def initialize(self, position: Tuple[float, float],
                   velocity: float, orientation: float) -> None:
        """Seed the filter with an initial state from the first sensor readings."""
        self._state = np.array([position[0], position[1], orientation, velocity],
                               dtype=float)
        self._cov = np.eye(4)
        self._initialized = True

    # ------------------------------------------------------------------
    # EKF prediction step (driven by IMU)
    # ------------------------------------------------------------------

    def predict(self, omega_z: float, accel_x: float, dt: float) -> None:
        """Propagate the state forward using IMU measurements.

        Args:
            omega_z:  Yaw rate from the IMU gyroscope in rad/s.
            accel_x:  Forward linear acceleration from the IMU accelerometer in m/s².
            dt:       Time-step in seconds since the last prediction.
        """
        if not self._initialized or dt <= 0.0:
            return

        x, y, theta, v = self._state

        # Non-linear state-transition equations
        theta_new = norm_angle(theta + omega_z * dt)
        v_new = v + accel_x * dt
        x_new = x + v * cos(theta) * dt
        y_new = y + v * sin(theta) * dt

        self._state = np.array([x_new, y_new, theta_new, v_new])

        # Jacobian of the state-transition function (linearised around current state)
        jac_f = np.array([
            [1.0, 0.0, -v * sin(theta) * dt,  cos(theta) * dt],
            [0.0, 1.0,  v * cos(theta) * dt,  sin(theta) * dt],
            [0.0, 0.0,  1.0,                  0.0            ],
            [0.0, 0.0,  0.0,                  1.0            ],
        ])

        self._cov = jac_f @ self._cov @ jac_f.T + self._proc_noise

    # ------------------------------------------------------------------
    # EKF update step (driven by odometry)
    # ------------------------------------------------------------------

    def update_odometry(self, position: Tuple[float, float], velocity: float) -> None:
        """Correct the state estimate using odometry measurements.

        Args:
            position: Measured (x, y) position from odometry.
            velocity: Measured forward speed from odometry in m/s.
        """
        if not self._initialized:
            return

        z = np.array([position[0], position[1], velocity])
        h = np.array([self._state[0], self._state[1], self._state[3]])

        # Linear observation matrix H for [x, y, v]
        obs_mat = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        self._apply_update(z - h, obs_mat, self._odom_noise)

    # ------------------------------------------------------------------
    # EKF update step (driven by IMU orientation)
    # ------------------------------------------------------------------

    def update_orientation(self, orientation: float) -> None:
        """Correct the heading estimate using the IMU quaternion-derived orientation.

        Args:
            orientation: Measured heading angle from the IMU in radians.
        """
        if not self._initialized:
            return

        innovation = np.array([norm_angle(orientation - self._state[2])])
        obs_mat = np.array([[0.0, 0.0, 1.0, 0.0]])

        self._apply_update(innovation, obs_mat, self._imu_noise)

    # ------------------------------------------------------------------
    # Internal helper
    # ------------------------------------------------------------------

    def _apply_update(self, innovation: np.ndarray,
                      obs_mat: np.ndarray,
                      meas_noise: np.ndarray) -> None:
        """Apply a generic Kalman update step."""
        innov_cov = obs_mat @ self._cov @ obs_mat.T + meas_noise
        kalman_gain = self._cov @ obs_mat.T @ np.linalg.inv(innov_cov)

        self._state = self._state + kalman_gain @ innovation
        self._state[2] = norm_angle(self._state[2])

        self._cov = (np.eye(4) - kalman_gain @ obs_mat) @ self._cov
