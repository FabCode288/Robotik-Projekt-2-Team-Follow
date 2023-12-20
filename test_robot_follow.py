import pytest
import numpy as np
from robot_follow import RobotFollow 

"""
Given a new RobotFollow with wanted dist 30 v 1 rot 1
When the method follow is called with invalid parameters
Then the method returns None
"""
def test_follow_invalid_parameters():
    mover = RobotFollow(30,1,1)
    assert mover.follow("a","b","c") == None

"""
Given a new RobotFollow with wanted dist 30 v 1 rot 1
When the method follow is called with pitch = 0 and roll = 0 and camera dist = 30
Then the method returns [1,0]
"""
def test_follow_30_1_1():
    mover = RobotFollow(30, 1,1)
    assert mover.follow(30,0,0) == [1,0]

"""
Given a new RobotFollow with wanted dist 30 v 1 rot 1
When the method follow is called with pitch = 0 and roll = 0 and camera dist = 20
Then the method returns [0.1,0]
"""
def test_follow_20_1_1():
    mover = RobotFollow(30, 1,1)
    assert mover.follow(20,0,0) == [0,0]

"""
Given a new RobotFollow with wanted dist 30 v 1 rot 1
When the method follow is called with pitch = 0 and roll = 0 and camera dist = 40
Then the method returns [0.1,0]
"""
def test_follow_40_1_1():
    mover = RobotFollow(30, 1,1)
    assert mover.follow(40,0,0) == [2,0]

