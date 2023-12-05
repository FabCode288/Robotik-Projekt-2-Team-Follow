from enum import Enum
import numpy as np
import math

class _State(Enum):
    RFID_DETECTED = 1
    MOVING_IN_PIPE = 2
    FOLLOWING = 3

class SimpleRobotMover:
    _omega = 0.3  # Rotationsgeschwindigkeit
    _v = 0.2  # Vorwärtsgeschwindigkeit

    def __init__(self):
        # Initialisierung des Roboters im Ruhezustand ohne vordefiniertes Ziel.
        self._state = _State.MOVING_IN_PIPE
        self._target_pose = None
        self._target_angle = 0
        
    def scan_for_RFID(self):
        return False

    def scan_for_Robot(self):
        return False

    def u_turn(self, pitch, roll, theta):
        _target_angle = theta + np.pi
        if(_target_angle > theta):
            return [0, SimpleRobotMover._omega]
        else:
            self._state = _State.MOVING_IN_PIPE                      
            return self.get_orientation_command(pitch, roll, theta)

    def get_orientation_command(self, pitch, roll, theta):
        """Berechnet die nächste Bewegung basierend auf der gesetzten Zielposition, der aktuellen Ausrichtung und dem aktuellen Zustand."""
        try:
            match self._state:
                case _State.RFID_DETECTED:#Wenden
                    return self.u_turn(pitch, roll, theta)
                case _State.MOVING_IN_PIPE:
                    if(self.scan_for_Robot()):
                        self._state = _State.FOLLOWING                        
                        return self.get_orientation_command(pitch, roll, theta)
                    elif(self.scan_for_RFID()):
                        self._state = _State.RFID_DETECTED
                        return self.get_orientation_command(pitch, roll, theta)
                    else:
                        return self.movment_pipe(pitch, roll)
                case _State.FOLLOWING:
                    # Der Roboter dreht sich in den finalen Rotationszustand.
                    return None
        except (ValueError, TypeError):
            return None


    def movment_pipe(self, pitch, roll):
        try:  
            if(abs(pitch) > abs(roll) + 0.1):
                if roll> 0:
                    return [0, -SimpleRobotMover._omega]
                else:
                    return [0, SimpleRobotMover._omega]
            else:
                return [cos(abs(roll)) * SimpleRobotMover._v, -1 * sin(roll) * SimpleRobotMover._omega]
        except (ValueError, TypeError):
            return None

    def follow_pipe(self, ):
        pass
