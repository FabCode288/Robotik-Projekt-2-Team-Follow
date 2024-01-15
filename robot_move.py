"""
Logic unit to controll the movement of a robot inside a pipe 
Makeing sure he heads towards the middle of the pipe or stops at an encountered RFID_tag or ArUco_marker
"""

import numpy as np
import math

class RobotMove:

    def __init__(self):
        self.last_dist = 0

    def follow_line(self, dist_to_line, v):#rechts positiv
        if abs(self.last_dist) < abs(dist_to_line)+5 and abs(dist_to_line) > 5:
            omega=dist_to_line*0.1 #faktor noch einstellen
            if abs(omega) > 0.5:
                omega = 0.5*(omega/abs(omega)) 
        elif self.last_dist-dist_to_line > 20:
            omega= -0.1*dist_to_line #faktor noch einstellen
            if abs(omega) > 0.5:
                omega = 0.5*(omega/abs(omega))            
        else:
            omega = 0  
        self.last_dist = dist_to_line
        return[v, omega] 
