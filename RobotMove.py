#Make the robot move inside the pipe until the rfid is reached or an aruco marker is detected

import numpy as np
import math

class RobotMove:

    def __init__(self, v, omega):
        self._v = v
        self._omega = omega

    def get_movement_pipe(self, pitch, roll):
        try:  
            if(abs(pitch) > abs(roll) + 0.1):
                if roll> 0:
                    return [0, -RobotMove._omega]
                else:
                    return [0, RobotMove._omega]
            else:
                return [math.cos(abs(roll)) * RobotMove._v, -1 * math.sin(roll) * RobotMove._omega]
        except (ValueError, TypeError):
            return None