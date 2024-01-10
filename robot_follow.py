import numpy as np
import math

class RobotFollow:

    def __init__(self, wanted_dist:
        self.wanted_dist = wanted_dist
        self.last_v = 0.1
        
    def follow(self, dist_to_robot, dist_to_line):
        res = (dist_to_robot-self.wanted_dist)*0.001 #Faktor noch einstellen
        if dist_to_robot <= 20:#stop
            return [0,0]
        else :#beschleunigen
            self.v += res
            return self.follow_line(dist_to_line, self.last_v)
        
    def follow_line(self, dist_to_line, v):#rechts positiv
        if abs(self.last_dist) < abs(dist_to_line)+5 and abs(dist_to_line) > 5:
            omega=dist_to_line*0.1 #faktor noch einstellen
            if abs(omega) < 0.5:
                return [v, omega]
            else:
                return[v, 0.5*(omega/abs(omega))]
        elif self.last_dist-dist_to_line>20:
            omega= -0.1*dist_to_line #faktor noch einstellen
            if abs(omega) < 0.5:
                return [v, omega]
            else:
                return[v, 0.5*(omega/abs(omega))]
        else:
            return[v, 0]
