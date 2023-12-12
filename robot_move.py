#Make the robot move inside the pipe until the rfid is reached or an aruco marker is detected

import numpy as np
import math

class RobotMove:

    def __init__(self):
        pass

    def get_movement_pipe(self, pitch, roll,v,omega,tag,camera_dist):
        """
        Checks for an RFID_tag or an ArUco_Marker 
        Ensures that the robot either stops its movement at encountering a RFID_tag or ArUco_marker or continues with its movement logic
        """
        try:  
            if(tag!="None" or camera_dist!= -1.0):
                return None
            else:
                return self.move_controll(pitch,roll,v,omega)
        except (ValueError, TypeError):
            return None
    def move_controll(self , pitch, roll, v, omega):
        """
        Calculating the linear and angular velocity for a robot inside a pipe based on its orientation and target_velocity
        Ensures that the Robot will head towards the middle of the pipe
        """
        if(abs(pitch) > abs(roll) + 0.1):
                if roll> 0:
                    return [0, -omega]
                else:
                    return [0, omega]
        else:
            return [math.cos(abs(roll)) * v, -1 * math.sin(roll) * omega]