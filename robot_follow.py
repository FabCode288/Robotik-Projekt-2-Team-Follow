import numpy as np
import math

class RobotFollow:

    def __init__(self, wanted_dist):
        self.wanted_dist = wanted_dist
        self.v = 0.1
        self.last_dist=0
    """
    calculates velocity command based on distance to robot ahead and target distance
    stops when threshold of 0.2m is reached
    increases or decreases velocity to match target distance
    angular velocity defined by follow_line()
    """    
    def follow(self, dist_to_robot, dist_to_line):
        try:
            res = (dist_to_robot-self.wanted_dist)*0.001 #Faktor noch einstellen
            if dist_to_robot <= 20:#stop
                return [0,0]
            else:
                self.v += res
                if (self.v<0):
                    self.v=0
                else:
                    self.v=min(self.v,0.1)
                return self.follow_line(dist_to_line, self.v)
        except:
            return None
        
    def follow_line(self, dist_to_line, v):#rechts positiv
        try:
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
        except:
            return None
