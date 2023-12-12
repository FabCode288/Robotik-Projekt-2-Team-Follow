#Follow the aruco marker in the given distance until no marker is detected, then idle

import numpy as np
import math

class RobotFollow:

    def __init__(self, wanted_dist, default_v, default_omega):
        self.wanted_dist = wanted_dist
        self.default_v = default_v
        self.default_omega = default_omega
        

    def follow(self, dist_to_robot, pitch, roll):
        res = (dist_to_robot-self.wanted_dist)*0.01 #Faktor noch einstellen
        return self.get_movement_pipe(pitch,roll, self.default_v+res*self.default_v, self.default_omega)


    def get_movement_pipe(self, pitch, roll, v, omega):
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
