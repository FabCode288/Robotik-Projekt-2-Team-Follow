import numpy as np
import math

class RobotTurn():
    def __init__(self, start_angle, default_omega):
        self.end_angle = start_angle + np.pi
        self.default_omega = default_omega

    def turn(self, current_angle):
        if current_angle < self.end_angle:
            return [0, self.default_omega]
        else:
            return [0,0]
        
        
       
        
        

            


        
 
        
        # mover = SimpleRobotMover()
        #mover.set_target_pose(goal_handle.request.pose.x, goal_handle.request.pose.y, goal_handle.request.pose.theta)
        #vel = mover.turn(self, _last_pose_pitch, _last_pose_roll) #inputs tbd
        # self.get_logger().info("Velocity1: {}, {}".format(vel[0], vel[1]))








 
