import numpy as np

class RobotTurn():
    def __init__(self, start_angle, default_omega):
        """
        Calculates the end_angle of half a rotation based on the start_angle
        angles in range -pi to pi
        passes for invalid parameters
        """
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
        returns None for invalid parameters
        """
        try:
            if abs(current_angle - self.end_angle) > 0.10:
                return [0, -self.default_omega]
            else:
                return None
        except:
            return None