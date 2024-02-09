"""
Logic unit to controll the movement of a robot inside a pipe 
Makeing sure he heads towards the middle of the line or stops at an encountered RFID_tag or ArUco_marker
"""

import math

class RobotMove:

    def __init__(self, max_omega):
        self.last_dist = 0
        self.max_omega = max_omega
        self.kP = 0.0002
        self.kD = 0.0002
        self.last_step = 0

    """
    Calculating the angular velocity based on the measured distance to the line
    intercepts the case of no line detected
    Increases or decreases angua velocity and intercepts too high velocities
    """

    def follow_line(self, dist_to_line, v, step):#rechts positiv
        if dist_to_line == 66666:
            return [v,0.0]
        else:
            if (step!=self.last_step):#avoiding divison by zero
                derivative = (dist_to_line - self.last_dist)/(step-self.last_step)
            else:
                derivative = 0
            self.last_step=step
                                                          
            omega = -1*(dist_to_line*self.kP + derivative*self.kD) 
            
            self.last_dist = dist_to_line
            if abs(omega) > self.max_omega:#zu Hohe Geschwindigkeit abfangen
                        omega = self.max_omega*math.copysign(1,omega) *-1
            return[v, omega] 
