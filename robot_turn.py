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
        
        
       
        
        

            


        
 
        
        # mover = SimpleRobotMover()
        #mover.set_target_pose(goal_handle.request.pose.x, goal_handle.request.pose.y, goal_handle.request.pose.theta)
        #vel = mover.turn(self, _last_pose_pitch, _last_pose_roll) #inputs tbd
        # self.get_logger().info("Velocity1: {}, {}".format(vel[0], vel[1]))
