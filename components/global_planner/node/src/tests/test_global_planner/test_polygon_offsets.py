from pytest import approx
from math import pi, sqrt
from global_planner.geometry import orthogonal_offset, bounding_box


def test_offset_north():
    offset = orthogonal_offset((0.0, 0.0), (0.0, 1.0), 2.0)
    assert offset == approx((2.0, 0.0), abs=1e-6)


def test_offset_south():
    offset = orthogonal_offset((0.0, 0.0), (0.0, -1.0), 2.0)
    assert offset == approx((-2.0, 0.0), abs=1e-6)


def test_offset_east():
    offset = orthogonal_offset((0.0, 0.0), (1.0, 0.0), 2.0)
    assert offset == approx((0.0, -2.0), abs=1e-6)


def test_offset_west():
    offset = orthogonal_offset((0.0, 0.0), (-1.0, 0.0), 2.0)
    assert offset == approx((0.0, 2.0), abs=1e-6)


def test_offset_north_east():
    offset = orthogonal_offset((0.0, 0.0), (1.0, 1.0), 2.0)
    assert offset == approx((sqrt(2.0), -sqrt(2.0)), abs=1e-6)


def test_offset_south_west():
    offset = orthogonal_offset((0.0, 0.0), (-1.0, -1.0), 2.0)
    assert offset == approx((-sqrt(2), sqrt(2)), abs=1e-6)


def test_bounding_box_north():
    points = bounding_box((0.0, 0.0), (0.0, 1.0), 2.0)
    assert points[0] == approx((2.0, 0.0), abs=1e-6)
    assert points[1] == approx((-2.0, 0.0), abs=1e-6)
    assert points[2] == approx((-2.0, 1.0), abs=1e-6)
    assert points[3] == approx((2.0, 1.0), abs=1e-6)


def test_bounding_box_south():
    points = bounding_box((0.0, 0.0), (0.0, -1.0), 2.0)
    assert points[0] == approx((-2.0, 0.0), abs=1e-6)
    assert points[1] == approx((2.0, 0.0), abs=1e-6)
    assert points[2] == approx((2.0, -1.0), abs=1e-6)
    assert points[3] == approx((-2.0, -1.0), abs=1e-6)


def test_bounding_box_east():
    points = bounding_box((0.0, 0.0), (1.0, 0.0), 2.0)
    assert points[0] == approx((0.0, -2.0), abs=1e-6)
    assert points[1] == approx((0.0, 2.0), abs=1e-6)
    assert points[2] == approx((1.0, 2.0), abs=1e-6)
    assert points[3] == approx((1.0, -2.0), abs=1e-6)


def test_bounding_box_west():
    points = bounding_box((0.0, 0.0), (-1.0, 0.0), 2.0)
    assert points[0] == approx((0.0, 2.0), abs=1e-6)
    assert points[1] == approx((0.0, -2.0), abs=1e-6)
    assert points[2] == approx((-1.0, -2.0), abs=1e-6)
    assert points[3] == approx((-1.0, 2.0), abs=1e-6)


def test_bounding_box_north_east():
    points = bounding_box((0.0, 0.0), (1.0, 1.0), 2.0)
    assert points[0] == approx((sqrt(2.0), -sqrt(2.0)), abs=1e-6)
    assert points[1] == approx((-sqrt(2.0), sqrt(2.0)), abs=1e-6)
    assert points[2] == approx((1.0-sqrt(2.0), 1.0+sqrt(2.0)), abs=1e-6)
    assert points[3] == approx((1.0+sqrt(2.0), 1.0-sqrt(2.0)), abs=1e-6)


def test_bounding_box_south_west():
    points = bounding_box((0.0, 0.0), (-1.0, -1.0), 2.0)
    assert points[0] == approx((-sqrt(2.0), sqrt(2.0)), abs=1e-6)
    assert points[1] == approx((sqrt(2.0), -sqrt(2.0)), abs=1e-6)
    assert points[2] == approx((-1.0 + sqrt(2.0), -1.0 - sqrt(2.0)), abs=1e-6)
    assert points[3] == approx((-1.0 - sqrt(2.0), -1.0 + sqrt(2.0)), abs=1e-6)
