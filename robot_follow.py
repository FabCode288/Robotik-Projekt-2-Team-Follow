import numpy as np

from move_server.robot_move import RobotMove
"""
Logic unit to controll a robot inside the pipe
makes sure the robot follows an ArUco marker and heads towards the line
"""
class RobotFollow(RobotMove):

    def __init__(self, wanted_dist):
        self.wanted_dist = wanted_dist
        self.v = 0
        self.last_dist=0
        self.kp2 = 0.005
        super().__init__(0.7)
    """
    calculates velocity command based on distance to robot ahead and target distance
    increases or decreases linear velocity to match target distance
    will slow down and stop when ArUco marker disappears
    angular velocity defined by follow_line()
    """    
    def follow(self, dist_to_robot, dist_to_line):
        try:
            error = (dist_to_robot-self.wanted_dist) 
           
            res = error * self.kp2            
            
            self.v += res
            if (self.v<=0):
                self.v=0
                return [0,0]
            else:
                self.v=min(self.v,0.1)

            return self.follow_line(dist_to_line, self.v)
        except:
            return None
        
