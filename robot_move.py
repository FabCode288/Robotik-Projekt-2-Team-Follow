"""
Logic unit to controll the movement of a robot inside a pipe 
Makeing sure he heads towards the middle of the pipe or stops at an encountered RFID_tag or ArUco_marker
"""

import numpy as np
import math

class RobotMove:

    def __init__(self):
        self.last_dist = 0
        pass
    """
    def get_movement_pipe(self, pitch, roll,v,omega,tag,camera_dist):
             Checks for an RFID_tag or an ArUco_Marker 
        Ensures that the robot either stops its movement at encountering a RFID_tag or ArUco_marker or continues with its movement logic
        
        try:  
            if(tag!="None" or camera_dist!= -1.0):
                return None
            else:
                return self.move_controll(pitch,roll,v,omega)
        except (ValueError, TypeError):
            return None
    def move_controll(self , pitch, roll, v, omega):
        """""""
        Calculating the linear and angular velocity for a robot inside a pipe based on its orientation and target_velocity
        Ensures that the Robot will head towards the middle of the pipe
        """"""
        if(abs(pitch) > abs(roll) + 0.1):
                if roll> 0:
                    return [0, -omega]
                else:
                    return [0, omega]
        else:
            return [math.cos(abs(roll)) * v, -1 * math.sin(roll) * omega]
    """
    def follow_line(self, dist_to_line, v):#rechts positiv
        if abs(self.last_dist) < abs(dist_to_line) and abs(dist_to_line) > 20:
            omega=dist_to_line*0.1 #faktor noch einstellen
            if abs(omega) < 0.5:
                return [v, omega]
            else:
                return[v, 0.5*(omega/abs(omega))]
        elif self.last_dist-dist_to_line>20
            omega= -0.1*dist_to_line #faktor noch einstellen
            if abs(omega) < 0.5:
                return [v, omega]
            else:
                return[v, 0.5*(omega/abs(omega))]
        else:
            return[v, 0]

        self.last_dist = dist_to_line
        
        
