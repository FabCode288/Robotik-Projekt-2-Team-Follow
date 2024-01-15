#test file for Robot Move
import pytest
import numpy as np

from move_server.robot_move import RobotMove

def test_follow_line_straight_path():
    robot = RobotMove()
    v = 0.1  

    # Test for straight path (dist_to_line close to 0)
    command = robot.follow_line(0, v)
    assert command[1] == 0, "Robot should move straight when on the line"

def test_follow_line_minor_adjustment():
    robot = RobotMove()
    v = 0.1

    # Test for minor adjustment (dist_to_line small but not 0)
    command = robot.follow_line(60, v)
    assert command[1] > 0, "Robot should adjust slightly for small deviations"

def test_follow_line_major_adjustment_towards_line():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = 450

    # Test for larger adjustment (dist_to_line large)
    command = robot.follow_line(500, v)
    assert command[1] == 0.5, "Robot should make major adjustments for large deviations"
def test_follow_line_negativ_major_adjustment_towards_line():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = -450

    # Test for larger adjustment (dist_to_line large)
    command = robot.follow_line(-500, v)
    assert command[1] == -0.5, "Robot should make major adjustments for large deviations"

def test_follow_line_negative_distance():
    robot = RobotMove()
    v = 0.1

    # Test for negative distance (should adjust in opposite direction)
    command = robot.follow_line(-60, v)
    assert command[1] < 0, "Robot should adjust in the opposite direction for negative distances"
def test_follow_line_adjustment_on_approach():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = 100

    # Test for negative distance (should adjust in opposite direction)
    command = robot.follow_line(80, v)
    assert command[1] < 0, "Robot should adjust in the opposite direction for negative distances"
def test_follow_line_adjustment_on_approach_negativ():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = -100

    # Test for negative distance (should adjust in opposite direction)
    command = robot.follow_line(-80, v)
    assert command[1] > 0, "Robot should adjust in the opposite direction for negative distances"


