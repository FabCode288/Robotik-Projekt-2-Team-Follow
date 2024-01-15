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
        if abs(self.last_dist-dist_to_line)<100:#Zu große Abweichungen abfangen
            dist_to_line = self.last_dist+50*(self.last_dist/abs(self.last_dist))#Alternativ mit geringerer gerichteter Abweichung weiter machen
        if abs(self.last_dist) < abs(dist_to_line)+10:#entfernt sich der Bot von der Linie?
            omega=dist_to_line*0.001 #faktor noch einstellen
            if abs(omega) > 0.5:#zu Hohe Geschwindigkeit abfangen
                omega = 0.5*(omega/abs(omega)) 
        elif abs(self.last_dist) > abs(dist_to_line)+10 and abs(dist_to_line)<100:#bewegt sich der Bot auf die Linie zu und ist nah dran?
            omega=dist_to_line*-0.001 #faktor noch einstellen
            if abs(omega) > 0.5:#zu Hohe Geschwindigkeit abfangen
                omega = 0.5*(omega/abs(omega)) 
        else:#Bot fährt auf der Linie
            omega = 0  
        self.last_dist = dist_to_line
        return[v, omega] 
