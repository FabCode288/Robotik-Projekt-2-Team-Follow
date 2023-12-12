import pytest
import numpy as np
from robot_follow import RobotFollow 

"""
Given a new RobotFollow with wanted dist 300 v 1 rot 1
When the method follow is called with invalid parameters
Then the method returns None
"""
def test_follow_invalid_parameters():
    mover = RobotFollow(300,1,1)
    assert mover.follow("a","b","c") == None

"""
Given a new RobotFollow with wanted dist 300 v 1 rot 1
When the method follow is called with pitch = 0 and roll = 0 and camera dist = 300
Then the method returns [1,0]
"""
def test_follow_300_1_1():
    mover = RobotFollow(300, 1,1)
    assert mover.follow(0,0,300) == [1,0]

"""
Given a new RobotFollow with wanted dist 300 v 1 rot 1
When the method follow is called with pitch = 0 and roll = 0 and camera dist = 200
Then the method returns [0.1,0]
"""
def test_follow_300_1_1():
    mover = RobotFollow(300, 1,1)
    assert mover.follow(0,0,200) == [0.1,0]

"""
Given a new RobotFollow with wanted dist 300 v 1 rot 1
When the method follow is called with pitch = 0 and roll = 0 and camera dist = 400
Then the method returns [0.1,0]
"""
def test_follow_300_1_1():
    mover = RobotFollow(300, 1,1)
    assert mover.follow(0,0,400) == [1.9,0]

