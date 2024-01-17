import numpy as np
import math
from move_server.robot_move import RobotMove
class RobotFollow(RobotMove):

    def __init__(self, wanted_dist):
        self.wanted_dist = wanted_dist
        self.v = 0.01
        self.last_dist=0
        self.kp2 = 0.005
        super().__init__()
    """
    calculates velocity command based on distance to robot ahead and target distance
    stops when threshold of 0.2m is reached
    increases or decreases velocity to match target distance
    angular velocity defined by follow_line()
    """    
    def follow(self, dist_to_robot, dist_to_line):
        try:
            error = (dist_to_robot-self.wanted_dist) #Faktor noch einstellen
           
            res = error * self.kp2 #Faktor noch einstellen
            
            if dist_to_robot <= 0.2: #Nothaltres
                print('Dist to Robot Logik' + str(dist_to_robot))
                return [0,0]
            else:
                self.v += res
                if (self.v<=0):
                    self.v=0
                    return [0,0]
                else:
                    self.v=min(self.v,0.1)

                return self.follow_line(dist_to_line, self.v)#[self.v,0]
        except:
            print('Logik returns None   ')
            return None
        
