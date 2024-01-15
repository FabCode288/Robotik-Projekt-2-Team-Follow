"""
Logic unit to controll the movement of a robot inside a pipe 
Makeing sure he heads towards the middle of the pipe or stops at an encountered RFID_tag or ArUco_marker
"""

import numpy as np
import math

class RobotMove:

    def __init__(self):
        self.last_dist = 0
        self.max_omega =0.5
        

    def follow_line(self, dist_to_line, v):#rechts positiv
        if abs(abs(self.last_dist)-abs(dist_to_line))>100:#Zu große Abweichungen abfangen
            dist_to_line = self.last_dist+50*math.copysign(1,self.last_dist)#Alternativ mit geringerer gerichteter Abweichung weiter machen
        if abs(dist_to_line)<100:#Bot ist nah an der Linie
            if abs(self.last_dist) < abs(dist_to_line): #entfernt sich der Bot von der Linie?
                omega=dist_to_line*0.001 #faktor noch einstellen
            elif abs(self.last_dist) > abs(dist_to_line):#bewegt sich der Bot auf die Linie zu?
                omega=dist_to_line*-0.001 #faktor noch einstellen
            else:#fährt parallel
                omega =0
        else:#Bot ist nicht nah an der Linie
            if abs(self.last_dist) < abs(dist_to_line)+10: #fährt der Bot auf die Linie zu?
                omega=dist_to_line*0.001 #faktor noch einstellen       
            else:#Bot fährt auf Linie zu
                omega = 0  

        self.last_dist = dist_to_line
        if abs(omega) > self.max_omega:#zu Hohe Geschwindigkeit abfangen
                    omega = self.max_omega*math.copysign(1,omega) 
        return[v, omega] 
