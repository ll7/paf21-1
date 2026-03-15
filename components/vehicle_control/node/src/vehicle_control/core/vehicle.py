"""This module represents a vehicle to be controlled"""

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np
from vehicle_control.core.geometry import rotate_vector, add_vector
from vehicle_control.core.sensor_fusion import SensorFusion
#     unit_vector, scale_vector, add_vector


@dataclass
class VehicleMetadata:
    """Representing construction-specific vehicle features"""
    length_between_axles_m: float = 3.1
    max_steer_angle_rad: float = np.deg2rad(40)
    base_accel_mps2: float = 2.0
    base_brake_mps2: float = -3.0
    wheelbase: float = 2.875
    vehicle_reaction_time_s: float = 0.1


@dataclass
class Vehicle:
    """Representing a vehicle with sensor-fused state estimation"""
    name: str
    velocity_mps: float = None
    steer_angle: float = 0
    pos: Tuple[float, float] = None
    orientation_rad: float = None
    time: float = 0.0
    meta: VehicleMetadata = field(default_factory=VehicleMetadata)
    sensor_fusion: SensorFusion = field(default_factory=SensorFusion)

    @property
    def is_ready(self) -> bool:
        """A boolean indicating whether the vehicle is ready for use"""
        return self.pos is not None and self.velocity_mps is not None and self.orientation_rad is not None

    # @property
    # def pos_front(self) -> Tuple[float, float]:
    #     """The vehicle's front axle position"""
    #     front_offset = scale_vector(unit_vector(self.orientation_rad), 1.5)
    #     return add_vector(self.pos, front_offset)

    def update_speed(self, speed):
        """function to update the current velocity of the car"""
        self.velocity_mps = speed

    def update_imu(self, omega_z: float, accel_x: float, dt: float):
        """Run the EKF prediction step using raw IMU gyroscope and accelerometer data.

        This is called at the IMU update rate (20 Hz) and propagates the state
        estimate forward in time between lower-frequency odometry updates.

        Args:
            omega_z: Yaw rate from the IMU gyroscope in rad/s.
            accel_x: Forward linear acceleration from the IMU accelerometer in m/s².
            dt:      Time elapsed since the last IMU reading in seconds.
        """
        self.sensor_fusion.predict(omega_z, accel_x, dt)

    def update_vehicle_state(self, position: Tuple[float, float],
                             velocity_and_time: Tuple[Tuple[float, float], float]):
        """Update the vehicle's positional and velocity values using sensor fusion.

        Odometry measurements are fed into the EKF as correction steps.  On the
        very first call the filter is also initialised so that subsequent IMU
        predictions have a valid starting point.

        ``self.orientation_rad`` is kept in sync from the EKF state after every
        call so that all readers (e.g. ``is_ready``) always see a consistent value.
        """
        if self.orientation_rad is None:
            return

        velocity, timestamp = velocity_and_time
        self.time = timestamp

        if not self.sensor_fusion.is_initialized:
            self.sensor_fusion.initialize(position, velocity, self.orientation_rad)
        else:
            self.sensor_fusion.update_odometry(position, velocity)

        axle_length = self.meta.wheelbase / 2
        vector_axle = rotate_vector((axle_length, 0), self.sensor_fusion.orientation_rad)
        self.pos = add_vector(vector_axle, self.sensor_fusion.position)
        self.velocity_mps = self.sensor_fusion.velocity_mps
        # Keep orientation_rad in sync with the EKF's fused heading
        self.orientation_rad = self.sensor_fusion.orientation_rad

    def update_vehicle_orientation(self, orientation: float):
        """Update the vehicle's heading using the IMU orientation measurement.

        The raw quaternion-derived orientation is fed into the EKF as a
        correction measurement.  Before the filter is initialised, the value is
        stored directly so it can be used to seed the filter on the first
        odometry update.

        ``self.orientation_rad`` is the single source of truth for heading; it
        mirrors ``sensor_fusion.orientation_rad`` once the filter is running.
        """
        if not self.sensor_fusion.is_initialized:
            self.orientation_rad = orientation
            return

        self.sensor_fusion.update_orientation(orientation)
        self.orientation_rad = self.sensor_fusion.orientation_rad
