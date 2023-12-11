#Make the robot move inside the pipe until the rfid is reached or an aruco marker is detected

import numpy as np
import math

class RobotMove:

    def __init__(self):
        pass

    def get_movement_pipe(self, pitch, roll,v,omega):
        try:  
            if(abs(pitch) > abs(roll) + 0.1):
                if roll> 0:
                    return [0, -omega]
                else:
                    return [0, omega]
            else:
                return [math.cos(abs(roll)) * v, -1 * math.sin(roll) * omega]
        except (ValueError, TypeError):
            return None
