"""
Logic unit to controll the movement of a robot inside a pipe 
Makeing sure he heads towards the middle of the pipe or stops at an encountered RFID_tag or ArUco_marker
"""

import numpy as np
import math

class RobotMove:

    def __init__(self):
        self.last_dist = 0
        self.max_omega =0.3
        self.kP = 0.0002
        

    def follow_line(self, dist_to_line, v):#rechts positiv
        if dist_to_line == 6666.0
            return [v,0.0]
        else:
            omega=-1*dist_to_line*self.kP #faktor noch einstellen   
            print("Distanz: " + str(dist_to_line) + " Omega: " +     str(omega))
    
    
            self.last_dist = dist_to_line
            if abs(omega) > self.max_omega:#zu Hohe Geschwindigkeit abfangen
                        omega = self.max_omega*math.copysign(1,omega) *-1
            return[v, omega] 
