#Follow the aruco marker in the given distance until no marker is detected, then idle

import numpy as np
import math

class RobotFollow:

    def __init__(self, wanted_dist, v):
        self.wanted_dist = wanted_dist
        self.v = v
        
    def follow(self, dist_to_robot, pitch, roll):
        if dist_to_robot > self.wanted_dist+0.02:#beschleunigen
            self.default_v += 0.05
            return self.get_movement_pipe(pitch, roll, self.default_v, self.default_omega)
        elif dist_to_robot <= self.wanted_dist:#stop
            return [0,0]
        else:#bremsen
            self.default_v -= 0.05
            return self.get_movement_pipe(pitch, roll, self.default_v, self.default_omega)
        

    def follow(self, dist_to_robot, dist_to_line):
        try:
            res = (dist_to_robot-self.wanted_dist)*0.1 #Faktor noch einstellen
            mov = self.follow_line(dist_to_line, self.v)
            mov[0] = mov[0]+res
            self.last_v = mov[0]
            return mov
        except (ValueError, TypeError):
            return None


    def follow_line(self, dist_to_line, v):#rechts positiv
        if abs(self.last_dist) < abs(dist_to_line)+5 and abs(dist_to_line) > 5:
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
