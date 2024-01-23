#test file for Robot Move
import pytest
import numpy as np

from move_server.robot_move import RobotMove

"""
Given a new RobotMove with v = 0.1 
When the method follow_line is called with invalid parameters
Then the method returns None
"""
def test_follow_line_invalild_parameters():
    robot = RobotMove()
    v = 0.1
    command = robot.follow_line("a","b")
    assert command == None

"""
Given a new RobotMove with v = 0.1 
When the method follow_line is called with dist to line = 0
Then the method returns omega = 0
"""
def test_follow_line_straight_path():
    robot = RobotMove()
    v = 0.1  

    # Test for straight path (dist_to_line close to 0)
    command = robot.follow_line(0, v)
    assert command[1] == 0, "Robot should move straight when on the line"
"""
Given a new RobotMove with v = 0.1 
When the method follow_line is called with dist to line = 66666
Then the method returns omega = 0
"""
def test_follow_no_line():
    robot = RobotMove()
    v = 0.1  
    command = robot.follow_line(0, v)
    assert command[1] == 0, "Robot should move straight when no line is detected line"

"""
Given a new RobotMove with v = 0.1 
When the method follow_line is called with dist to line = 60
Then the method returns omega > 0
"""
def test_follow_line_minor_adjustment():
    robot = RobotMove()
    v = 0.1

    # Test for minor adjustment (dist_to_line small but not 0)
    command = robot.follow_line(60, v)
    assert command[1] > 0, "Robot should adjust slightly for small deviations"

"""
Given a new RobotMove with v = 0.1
When the method follow_line is called with dist to line = -60
Then the method returns omega < 0
"""
def test_follow_line_negative_distance():
    robot = RobotMove()
    v = 0.1

    # Test for negative distance (should adjust in opposite direction)
    command = robot.follow_line(-60, v)
    assert command[1] < 0, "Robot should adjust in the opposite direction for negative distances"

"""
Given a new RobotMove with v = 0.1 and last dist = 450
When the method follow_line is called with dist to line = 500
Then the method returns omega == 0.5
"""
def test_follow_line_major_adjustment_towards_line():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = 450

    # Test for larger adjustment (dist_to_line large)
    command = robot.follow_line(500, v)
    assert command[1] == 0.5, "Robot should make major adjustments for large deviations"

"""
Given a new RobotMove with v = 0.1 and last dist = -450
When the method follow_line is called with dist to line = -500
Then the method returns omega == -0.5
"""
def test_follow_line_negativ_major_adjustment_towards_line():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = -450

    # Test for larger adjustment (dist_to_line large)
    command = robot.follow_line(-500, v)
    assert command[1] == -0.5, "Robot should make major adjustments for large deviations"

"""
Given a new RobotMove with v = 0.1 and last dist = 100
When the method follow_line is called with dist to line = 80
Then the method returns omega < 0
"""
def test_follow_line_adjustment_on_approach():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = 100

    # Test for negative distance (should adjust in opposite direction)
    command = robot.follow_line(80, v)
    assert command[1] < 0, "Robot should adjust in the opposite direction for negative distances"

"""
Given a new RobotMove with v = 0.1 and last dist = -100
When the method follow_line is called with dist to line = -80
Then the method returns omega > 0
"""
def test_follow_line_adjustment_on_approach_negativ():
    robot = RobotMove()
    v = 0.1
    robot.last_dist = -100

    # Test for negative distance (should adjust in opposite direction)
    command = robot.follow_line(-80, v)
    assert command[1] > 0, "Robot should adjust in the opposite direction for negative distances"


