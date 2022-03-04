from pytest import approx
from math import pi, sqrt
from global_planner.geometry import orth_offset_right, bounding_box, is_below_line


def test_offset_north():
    offset = orth_offset_right((0.0, 0.0), (0.0, 1.0), 2.0)
    assert offset == approx((2.0, 0.0), abs=1e-6)


def test_offset_south():
    offset = orth_offset_right((0.0, 0.0), (0.0, -1.0), 2.0)
    assert offset == approx((-2.0, 0.0), abs=1e-6)


def test_offset_east():
    offset = orth_offset_right((0.0, 0.0), (1.0, 0.0), 2.0)
    assert offset == approx((0.0, -2.0), abs=1e-6)


def test_offset_west():
    offset = orth_offset_right((0.0, 0.0), (-1.0, 0.0), 2.0)
    assert offset == approx((0.0, 2.0), abs=1e-6)


def test_offset_north_east():
    offset = orth_offset_right((0.0, 0.0), (1.0, 1.0), 2.0)
    assert offset == approx((sqrt(2.0), -sqrt(2.0)), abs=1e-6)


def test_offset_south_west():
    offset = orth_offset_right((0.0, 0.0), (-1.0, -1.0), 2.0)
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


def test_is_below_line():
    # positive steem (left / mid / right with one pos / neg case each)
    assert is_below_line((1, 1), (3, 2), (0, 0.49)) == True
    assert is_below_line((1, 1), (3, 2), (0, 0.51)) == False
    assert is_below_line((1, 1), (3, 2), (2, 1.49)) == True
    assert is_below_line((1, 1), (3, 2), (2, 1.51)) == False
    assert is_below_line((1, 1), (3, 2), (4, 2.49)) == True
    assert is_below_line((1, 1), (3, 2), (4, 2.51)) == False

    # negative steem (left / mid / right with one pos / neg case each)
    assert is_below_line((1, 2), (3, 1), (4, 0.49)) == True
    assert is_below_line((1, 2), (3, 1), (4, 0.51)) == False
    assert is_below_line((1, 2), (3, 1), (2, 1.49)) == True
    assert is_below_line((1, 2), (3, 1), (2, 1.51)) == False
    assert is_below_line((1, 2), (3, 1), (0, 2.49)) == True
    assert is_below_line((1, 2), (3, 1), (0, 2.51)) == False
