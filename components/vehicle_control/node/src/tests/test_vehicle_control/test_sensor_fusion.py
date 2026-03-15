"""Tests for the SensorFusion EKF module"""

from math import pi, cos, sin, isclose
import pytest
from pytest import approx

from vehicle_control.core.sensor_fusion import SensorFusion


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_initialized_filter(x=0.0, y=0.0, theta=0.0, v=0.0) -> SensorFusion:
    sf = SensorFusion()
    sf.initialize((x, y), v, theta)
    return sf


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

def test_initialization_sets_state():
    sf = make_initialized_filter(x=10.0, y=5.0, theta=pi / 4, v=3.0)
    assert sf.position == approx((10.0, 5.0), abs=1e-9)
    assert sf.orientation_rad == approx(pi / 4, abs=1e-9)
    assert sf.velocity_mps == approx(3.0, abs=1e-9)
    assert sf.is_initialized is True

def test_not_initialized_by_default():
    sf = SensorFusion()
    assert sf.is_initialized is False


# ---------------------------------------------------------------------------
# Predict (IMU motion model)
# ---------------------------------------------------------------------------

def test_predict_straight_motion():
    """Driving straight east (theta=0) with constant velocity, no yaw rate."""
    sf = make_initialized_filter(x=0.0, y=0.0, theta=0.0, v=10.0)
    sf.predict(omega_z=0.0, accel_x=0.0, dt=1.0)
    x, y = sf.position
    assert x == approx(10.0, abs=1e-6)
    assert y == approx(0.0, abs=1e-6)
    assert sf.velocity_mps == approx(10.0, abs=1e-6)


def test_predict_applies_acceleration():
    """Accelerating from rest – velocity should increase."""
    sf = make_initialized_filter(x=0.0, y=0.0, theta=0.0, v=0.0)
    sf.predict(omega_z=0.0, accel_x=5.0, dt=2.0)
    assert sf.velocity_mps == approx(10.0, abs=1e-6)


def test_predict_applies_yaw_rate():
    """Yaw rate changes the heading angle."""
    sf = make_initialized_filter(theta=0.0)
    sf.predict(omega_z=pi / 2, accel_x=0.0, dt=1.0)
    assert sf.orientation_rad == approx(pi / 2, abs=1e-6)


def test_predict_angle_wraps_at_pi():
    """Heading angle stays within [-pi, pi)."""
    sf = make_initialized_filter(theta=pi - 0.1)
    sf.predict(omega_z=0.5, accel_x=0.0, dt=1.0)
    assert -pi <= sf.orientation_rad < pi


def test_predict_does_nothing_before_initialization():
    sf = SensorFusion()
    sf.predict(omega_z=1.0, accel_x=1.0, dt=0.1)
    assert sf.is_initialized is False


def test_predict_does_nothing_for_zero_dt():
    sf = make_initialized_filter(x=5.0, y=3.0, theta=0.0, v=10.0)
    sf.predict(omega_z=0.1, accel_x=2.0, dt=0.0)
    assert sf.position == approx((5.0, 3.0), abs=1e-9)
    assert sf.velocity_mps == approx(10.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Update – Odometry
# ---------------------------------------------------------------------------

def test_update_odometry_corrects_position():
    """Odometry update pulls estimated position towards the measurement."""
    # Start the filter at (5, 5) travelling at 0 m/s
    sf = make_initialized_filter(x=5.0, y=5.0, theta=0.0, v=0.0)
    # Odometry says we are at (0, 0) – the update should pull the estimate towards (0, 0)
    sf.update_odometry(position=(0.0, 0.0), velocity=0.0)
    x, y = sf.position
    assert abs(x) < 5.0
    assert abs(y) < 5.0


def test_update_odometry_corrects_velocity():
    sf = make_initialized_filter(x=0.0, y=0.0, theta=0.0, v=20.0)
    sf.update_odometry(position=(0.0, 0.0), velocity=0.0)
    assert sf.velocity_mps < 20.0


def test_update_odometry_does_nothing_before_initialization():
    sf = SensorFusion()
    sf.update_odometry(position=(1.0, 2.0), velocity=5.0)
    assert sf.is_initialized is False


# ---------------------------------------------------------------------------
# Update – Orientation (IMU)
# ---------------------------------------------------------------------------

def test_update_orientation_corrects_heading():
    # Initialise with theta=0.5, then correct towards 0.0
    sf = make_initialized_filter(theta=0.5)
    sf.update_orientation(0.0)
    assert abs(sf.orientation_rad) < 0.5


def test_update_orientation_keeps_angle_in_range():
    sf = make_initialized_filter(theta=pi - 0.1)
    sf.update_orientation(-pi + 0.1)
    assert -pi <= sf.orientation_rad < pi


def test_update_orientation_does_nothing_before_initialization():
    sf = SensorFusion()
    sf.update_orientation(1.0)
    assert sf.is_initialized is False


# ---------------------------------------------------------------------------
# Combined predict + update cycle
# ---------------------------------------------------------------------------

def test_predict_then_update_stays_consistent():
    """A full predict→update cycle should keep the estimate sensible."""
    sf = make_initialized_filter(x=0.0, y=0.0, theta=0.0, v=10.0)
    sf.predict(omega_z=0.0, accel_x=0.0, dt=0.05)
    sf.update_odometry(position=(0.5, 0.0), velocity=10.0)
    sf.update_orientation(0.0)
    # Position should be close to 0.5 m after 0.05 s at 10 m/s
    x, y = sf.position
    assert x == approx(0.5, abs=0.1)
    assert abs(y) < 0.1


def test_multiple_predict_steps_accumulate_position():
    """Running several prediction steps should advance position proportionally."""
    sf = make_initialized_filter(x=0.0, y=0.0, theta=0.0, v=10.0)
    for _ in range(10):
        sf.predict(omega_z=0.0, accel_x=0.0, dt=0.1)
    x, y = sf.position
    assert x == approx(10.0, abs=0.01)
    assert abs(y) < 0.01


def test_covariance_grows_during_prediction_shrinks_on_update():
    """Covariance should grow after predictions and shrink after updates."""
    sf = make_initialized_filter(x=0.0, y=0.0, theta=0.0, v=5.0)
    initial_trace = sf._cov.trace()
    sf.predict(omega_z=0.0, accel_x=0.0, dt=0.1)
    after_predict_trace = sf._cov.trace()
    assert after_predict_trace > initial_trace

    sf.update_odometry(position=(0.5, 0.0), velocity=5.0)
    after_update_trace = sf._cov.trace()
    assert after_update_trace < after_predict_trace
