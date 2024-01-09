#Follow the aruco marker in the given distance until no marker is detected, then idle

import numpy as np
import math

class RobotFollow:

    def __init__(self, wanted_dist, v):
        self.wanted_dist = wanted_dist
        self.last_v = v
        
    def follow(self, dist_to_robot, dist_to_line):
        if dist_to_robot <= 0.2:#stop
            return [0,0]
        elif dist_to_robot > self.wanted_dist+0.02:#beschleunigen
            self.v += 0.05
            return self.follow_line(dist_to_line, self.v)
        elif dist_to_robot < self.wanted_dist-0.02:#beschleunigen
            self.v += 0.05
            return self.follow_line(dist_to_line, self.v)
        else:#weitermachen
            return self.follow_line(dist_to_line, self.v)
        

    def follow(self, dist_to_robot, dist_to_line):
        try:
            res = (dist_to_robot-self.wanted_dist)*0.1 #Faktor noch einstellen
            mov = self.follow_line(dist_to_line, self.last_v+res)
            return mov
            self.last_v = self.last_v+res
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
