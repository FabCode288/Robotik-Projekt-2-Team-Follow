#test file for Robot Move
import pytest
import numpy as np

from insert_dir import RobotMove

def test_follow_line_straight_path():
    robot = RobotMove()
    v = 0.1  

    # Test for straight path (dist_to_line close to 0)
    command = robot.follow_line(0, v)
    assert command == [v, 0], "Robot should move straight when on the line"

def test_follow_line_minor_adjustment():
    robot = RobotMove()
    v = 0.1

    # Test for minor adjustment (dist_to_line small but not 0)
    command = robot.follow_line(6, v)
    assert command[0] == v and abs(command[1]) > 0, "Robot should adjust slightly for small deviations"

def test_follow_line_major_adjustment():
    robot = RobotMove()
    v = 0.1

    # Test for larger adjustment (dist_to_line large)
    command = robot.follow_line(50, v)
    assert command[0] == v and abs(command[1]) == 0.5, "Robot should make major adjustments for large deviations"

def test_follow_line_negative_distance():
    robot = RobotMove()
    v = 0.1

    # Test for negative distance (should adjust in opposite direction)
    command = robot.follow_line(-10, v)
    assert command[0] == v and command[1] == -0.1, "Robot should adjust in the opposite direction for negative distances"

def test_follow_line_with_last_dist():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = 15

    # Test behavior with a non-zero last_dist
    command = robot.follow_line(5, v)
    # Assert based on expected behavior considering last_dist
    # This depends on how you expect last_dist to affect the behavior
