#Start Befehl
#ros2 run ro36_simple_mover_server Ro36SimpleMoverServer
#Send empty Msg
#ros2 action send_goal -f /go_to ro36_interfaces/action/GoTo "{}"
#Send full Msg
#ros2 action send_goal -f /move ro36_interfaces/action/GoTo "{pose: {x: 4.0, y: 4.0, theta: 0.0}}"
#source install/setup.bash

import rclpy
import math
import time
import threading

from rclpy.action import ActionServer
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32

from ro36_interfaces.action import Follow
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from follow_server.robot_follow import RobotFollow

class FollowServer(Node):

    def __init__(self):
        super().__init__('follow_action_server')
        self.dist_to_line = 0
        self.dist_to_robot = 0
        self.i=0

        self.robot_dist_sub = self.create_subscription(
            Float32,
            'aruco_distance',
            self._robot_dist_callback,
            10)

        self.line_dist_sub = self.create_subscription(
            Float32,
            'line_distance',
            self._line_dist_callback,
            10)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._goal_handle = None

        self._action_server = ActionServer(
            self,
            Follow,
            'follow',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self._goal_handle_lock = threading.Lock() #Sorgt dafür dass die Action nur von einem Thread gleichzeitig ausgeführt wird und nicht von mehreren parallel 

    def _robot_dist_callback(self, msg):
        self.dist_to_robot = msg.data

    def _line_dist_callback(self, msg):
        self.dist_to_line = msg.data


    def _goal_callback(self, goal_request):
        self.get_logger().info('Received goal request with target distance')
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_handle_lock: #locks the next block for another thread, just one thread can access it
            if (self._goal_handle is not None and self._goal_handle.is_active):
                self.get_logger().info('Replace active goal with new goal.')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelling follow')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing follow')
        self.get_logger().info("Distance to Robot: {}".format(self.dist_to_robot))
        mover = RobotFollow(goal_handle.request.target_distance)
        self.get_logger().info('Target Dist: ' + str(goal_handle.request.target_distance))

        vel = mover.follow(self.dist_to_robot, self.dist_to_line) 
        print('vel: ' + str(vel))
        if vel is not None: self.get_logger().info("Velocity first: {}, {}".format(vel[0], vel[1])) #zu distance ändern
        self.i=0
        while(goal_handle.is_active and not goal_handle.is_cancel_requested): #and self.i<100):#if vel is None, wait for 50 Iterations before stopping, to compensate for lost frames
            vel = mover.follow(self.dist_to_robot, self.dist_to_line) 
            #self.get_logger().info('Executing While')

            if self.dist_to_robot==-1.0:
               self.i += 1
            else:
                print('vel: ' + str(vel))

                print('Dist_to robot while: ' + str(self.dist_to_robot))
                self._publish_velocity(vel)
                self._publish_feedback(goal_handle,vel)
            self.i = 0
            time.sleep(0.05)
                
        self._publish_velocity(None)
        return self._determine_action_result(goal_handle)

    def _publish_velocity(self, vel):
        velocity_msg = Twist() 
        if(vel is not None):
            self.get_logger().info("Velocity: {}, {}".format(vel[0], vel[1]) )
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0]) 
        else:
            velocity_msg.angular.z = float(0.0)
            velocity_msg.linear.x = float(0.0) 
        self._cmd_pub.publish(velocity_msg) 

    def _publish_feedback(self, goal_handle, vel):
        if(vel is not None):
            feedback_msg = Follow.Feedback()
            velocity_msg = Twist()            
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0]) 
            feedback_msg.current_velocity = velocity_msg
            goal_handle.publish_feedback(feedback_msg)

    def _determine_action_result(self, goal_handle):
        result = Follow.Result()
        if goal_handle.is_active and self.i>=50:
            self.get_logger().info('Follow succeeded') 
            goal_handle.succeed()
            #result.reached = True
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Follow was canceled')
        else:
            if goal_handle.is_active:
                goal_handle.abort()
                self.get_logger().info('Follow was aborted')
        self.i = 0
        return result


def main():
    rclpy.init()
    robot_mover_executor = MultiThreadedExecutor(10)
    follow_server = FollowServer()
    try:
        rclpy.spin(node=follow_server, executor=robot_mover_executor)
    finally:
        follow_server.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
