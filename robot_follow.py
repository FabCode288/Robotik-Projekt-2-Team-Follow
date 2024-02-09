from move_server.robot_move import RobotMove

class RobotFollow(RobotMove):

    def __init__(self, wanted_dist):
        if(wanted_dist > 1): #catching too high distances
            wanted_dist = 1
        if(wanted_dist < 0.2):#catching too low distances
            wanted_dist = 0.2
        self.wanted_dist = wanted_dist
        self.v = 0.01
        self.last_dist = 0
        self.kp = 0.005

        super().__init__(0.7)
    """
    calculates velocity command based on distance to robot ahead and target distance
    stops when threshold of 0.2m is reached
    increases or decreases velocity to match target distance
    angular velocity defined by follow_line()
    Using a PT1 system with a simple P regulating algorithm
    """    
    def follow(self, dist_to_robot, dist_to_line, step):
        try:
            dist_error = (dist_to_robot-self.wanted_dist) #Distance difference
            p = dist_error * self.kp       

            self.v += p #accelerate and brake
            if (self.v <= 0): #exclude negative values ​​so as not to drive backwards
                self.v = 0
                return [0, 0]
            else:
                self.v = min(self.v, 0.1) #set a maximal velocity

            return self.follow_line(dist_to_line, self.v, step) #follow line from robot_move
        except:
            return None
        
