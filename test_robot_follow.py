import pytest
import numpy as np
from robot_follow import RobotFollow 

"""
Given a new RobotFollow with wanted dist 30 dist to line 0
When the method follow is called with invalid parameters
Then the method returns None
"""
def test_follow_invalid_parameters():
    mover = RobotFollow(30)
    command = mover.follow("a","b")
    assert command == None

"""
Given a new RobotFollow with wanted dist 30 
When the method follow is called with parameters 30 0
Then the method returns [0.1, 0]
"""
def test_follow_30_0():
    mover = RobotFollow(30)
    command = mover.follow(30,0)
    assert command[0] == 0.1
    assert command[1] == 0

"""
Given a new RobotFollow with wanted dist 30 
When the method follow is called with parameters 40 0
Then the method returns [>0.1, 0]
"""
def test_follow_40_0():
    mover = RobotFollow(30)
    command = mover.follow(40,0)
    assert command[0] > 0.1
    assert command[1] == 0

"""
Given a new RobotFollow with wanted dist 30 
When the method follow is called with parameters 25 0
Then the method returns [<0.1, 0]
"""
def test_follow_25_0():
    mover = RobotFollow(30)
    command = mover.follow(25,0)
    assert command[0] < 0.1
    assert command[1] == 0

"""
Given a new RobotFollow with wanted dist 30 
When the method follow is called with parameters 10 0
Then the method returns [0, 0]
"""
def test_follow_10_0():
    mover = RobotFollow(30)
    command = mover.follow(10,0)
    assert command[0] == 0
    assert command[1] == 0


