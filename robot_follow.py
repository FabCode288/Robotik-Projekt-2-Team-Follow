#Follow the aruco marker in the given distance until no marker is detected, then idle

import numpy as np
import math

class RobotFollow:

    def __init__(self, wanted_dist, v, dist_to_line):
        self.wanted_dist = wanted_dist
        self.v = v
        self.dist_to_line = dist_to_line
        
        

    def follow(self, dist_to_robot):
        try:
            res = (dist_to_robot-self.wanted_dist)*0.1 #Faktor noch einstellen
            mov = self.follow_line(self.dist_to_line, self.v)
            mov[0] = mov[0]+res
            self.last_v = mov[0]
            return mov
        except (ValueError, TypeError):
            return None


    def follow_line(self, dist_to_line, v):  #selbe methode wie in move
        pass
