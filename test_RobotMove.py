#test file for Robot Move
import pytest
import numpy as np

from insert_dir import RobotMove
"""
Given a new RobotMove
When the method get_movement_pipe is called with invalid parameters
Then the method returns None
"""
def test_gmp_invalid_parameters():
    mover = RobotMove
    assert mover.get_movement_pipe(None) == None
    
"""
Given a new RobotMove
When the method get_movement_pipe is called with pitch = 0 and roll = 0
Then the method returns [RobotMove._v, 0]
"""

def test_gmp_0_0():
    mover = RobotMove
    assert mover.get_movement_pipe(0,0) == [RobotMove._v, 0]

"""
Given a new RobotMove
When the method get_movement_pipe is called with pitch = pi/2 and roll = 0
Then the method returns [0, RobotMove._omega]
"""

def test_gmp_pi2_0():
    mover = RobotMove
    assert mover.get_movement_pipe(np.pi/2,0) == [0, RobotMove._omega]

"""
Given a new RobotMove
When the method get_movement_pipe is called with pitch = 0 and roll = pi/2
Then the method returns [0, -RobotMove._omega]
"""

def test_gmp_0_pi2():
    mover = RobotMove
    assert mover.get_movement_pipe(0, np.pi/2) == [0, -RobotMove._omega]   

"""
Given a new RobotMove
When the method get_movement_pipe is called with pitch = pi/2 and roll = pi/2
Then the method returns [0, -RobotMove._omega]
"""

def test_gmp_pi2_pi2():
    mover = RobotMove
    assert mover.get_movement_pipe(np.pi/2,np.pi/2) == [0, -RobotMove._omega]

"""
Given a new RobotMove
When the method get_movement_pipe is called with pitch = pi/2 and roll = 1
Then the method returns [0, -RobotMove._omega]
"""

def test_gmp_pi2_1():
    mover = RobotMove
    assert mover.get_movement_pipe(np.pi/2,1) == [0, -RobotMove._omega]

"""
Given a new RobotMove
When the method get_movement_pipe is called with pitch = pi/2 and roll = -1
Then the method returns [0, RobotMove._omega]
"""

def test_gmp_pi2_minus1():
    mover = RobotMove
    assert mover.get_movement_pipe(np.pi/2,-1) == [0, RobotMove._omega]
