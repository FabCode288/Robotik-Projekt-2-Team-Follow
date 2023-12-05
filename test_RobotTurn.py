import pytest
import numpy as np
from ro36_simple_mover_server.RobotTurn import RobotTurn

"""The Robot starts at 0 degrees and is currently at 0 Degrees, it did not turn yet. We expect an rotating velocity > 0"""
def test_turn_start_angle_0_current_angle_0():
    turner = RobotTurn(0, 1) #The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(0) == [0, 1] # The current angle of the Robot while rotating

"""The Robot starts at 0 degrees and is currently at 45 Degrees, it has already partialy turned. We expect an rotating velocity > 0"""
def test_turn_start_angle_0_current_angle_45():
    turner = RobotTurn(0, 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(45)) == [0, 1] # The current angle of the Robot while rotating

"""The Robot starts at 0 degrees and is currently at 180 Degrees, it has already turned completely. We expect an rotating velocity = 0"""
def test_turn_start_angle_0_current_angle_180():
    turner = RobotTurn(0, 1)#The Angle of the Robot at the RFID, Rotating Velocity
    print(np.deg2rad(180))
    assert turner.turn(np.deg2rad(180)) == [0, 0] # The current angle of the Robot while rotating

"""The Robot starts at 0 Degrees and rotates, but dont stop at 180 degrees cause by a bug. Then it should rotate backward. We expect an rotating velocity < 0"""
# Achtung  ###### Hier wird aktuell noch ein fehler geschmissen, weil der Roboter sich zu weit gedreht hat########
def test_turn_start_angle_0_current_angle_200():    
    turner = RobotTurn(0, 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(200)) == [0, -1] # The current angle of the Robot while rotating

"""The Robot starts at -30 degrees and is currently at -30 Degrees, it did not turn yet. We expect an rotating velocity > 0"""
def test_turn_start_angle_minus_30_current_angle_minus_30():
    turner = RobotTurn(np.radians(-30), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(-30)) == [0, 1] # The current angle of the Robot while rotating

"""The Robot starts at -30 degrees and is currently at 70 Degrees, it has already partialy turned. We expect an rotating velocity > 0"""
def test_turn_start_angle_minus_30_current_angle_70():
    turner = RobotTurn(np.radians(-30), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(70)) == [0, 1] # The current angle of the Robot while rotating

"""The Robot starts at -180 degrees and is currently at -80 Degrees, it has already partialy turned. We expect an rotating velocity > 0"""
def test_turn_start_angle_minus_180_current_angle_minus_80():
    turner = RobotTurn(np.radians(-180), 1)#The Angle of the Robot at the RFID, Rotating Velocity
    assert turner.turn(np.radians(-80)) == [0, 1] # The current angle of the Robot while rotating
