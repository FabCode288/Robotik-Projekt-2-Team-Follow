import numpy as np
import math
#fertiggestellt
class RobotTurn():
    def __init__(self, start_angle, default_omega):
        try:
            if(start_angle <= 0):
                self.end_angle = start_angle + np.pi
            else:
                self.end_angle = start_angle - np.pi
            self.default_omega = default_omega    
        except:
            pass

    def turn(self, current_angle):
        """
        Checks if the desired target angle is reached
        If not, it returns the velocity to turn with,
        else it returns None
        """
        try:
            if abs(current_angle - self.end_angle) > 0.05:
                return [0, -self.default_omega]
            else:
                return [0,0]
        except:
            return None
        
    
