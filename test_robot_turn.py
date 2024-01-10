import pytest
import numpy as np
from turn_server.TurnServer import RobotTurn

"""The Robot starts at 0 degrees and is currently at 0 Degrees, it did not turn yet. We expect an rotating velocity < 0"""
def test_turn_start_angle_0_current_angle_0():
    turner = RobotTurn(0, 1) #The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(0) == [0, -1] # The current angle of the Robot while rotating

"""The Robot starts at 0 degrees and is currently at 45 Degrees, it has already partialy turned. We expect an rotating velocity < 0"""
def test_turn_start_angle_0_current_angle_45():
    turner = RobotTurn(0, 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(45)) == [0, -1] # The current angle of the Robot while rotatings

"""The Robot starts at 0 degrees and is currently at 180 Degrees, it has already turned completely. We expect an rotating velocity = 0"""
def test_turn_start_angle_0_current_angle_180():
    turner = RobotTurn(np.radians(0), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(180)) == None # The current angle of the Robot while rotating

"""The Robot starts at 160 degrees and is currently at -20 Degrees, it has already turned completely. We expect an rotating velocity = 0"""
def test_turn_start_angle_160_current_angle_180():
    turner = RobotTurn(np.radians(160), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(-20)) == None # The current angle of the Robot while rotating

"""The Robot starts at -30 degrees and is currently at -30 Degrees, it did not turn yet. We expect an rotating velocity < 0"""
def test_turn_start_angle_minus_30_current_angle_minus_30():
    turner = RobotTurn(np.radians(-30), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(-30)) == [0, -1] # The current angle of the Robot while rotating

"""The Robot starts at -30 degrees and is currently at 70 Degrees, it has already partialy turned. We expect an rotating velocity < 0"""
def test_turn_start_angle_minus_30_current_angle_70():
    turner = RobotTurn(np.radians(-30), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(70)) == [0, -1] # The current angle of the Robot while rotating

"""The Robot starts at -180 degrees and is currently at -80 Degrees, it has already partialy turned. We expect an rotating velocity < 0"""
def test_turn_start_angle_minus_180_current_angle_minus_80():
    turner = RobotTurn(np.radians(-180), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(-80)) == [0, -1] # The current angle of the Robot while rotating
"""The Robot gets incorrect parameters, We expect None"""
def test_turn_wrong_parameters():
    turner = RobotTurn("b","c")
    assert turner.turn("a") == None 
