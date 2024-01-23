import rclpy
import time
import threading

from std_msgs.msg import String
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from ro36_interfaces.action import Move
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from move_server.robot_move import RobotMove

import signal
import sys

"""
Action server to controll movement of a robot which follows a line
Has subscriptions on the RFID sensor, the distance to the line and the distance to an ArUco Marker ahead
Publishes movement orders to the robot 
"""

class MoveServer(Node):

    def __init__(self):
        super().__init__('move_action_server')
        self._last_rfid_tag = "None"
        self._new_rfid_tag = "None"
        self._dist_to_line = 66666
        self.dist_to_robot = -1.0

        self.rfid_sub = self.create_subscription(
            String,
            'rfid_topic',
            self._rfid_callback,
            10)
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
            Move,
            'move',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self._goal_handle_lock = threading.Lock() #Sorgt dafür dass die Action nur von einem Thread gleichzeitig ausgeführt wird und nicht von mehreren parallel 

    def _rfid_callback(self, msg):  
        self._last_rfid_tag = msg.data   
       
    def _line_dist_callback(self, msg):
        self._dist_to_line = msg.data   

    def _robot_dist_callback(self, msg):
        self.dist_to_robot = msg.data

    def _goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    """
    Returns true when an new RFID-tag is reached 
    and False for no or the same tag beeing detected
    """    
    def rfid_changed(self):
        if (self._last_rfid_tag != self._new_rfid_tag and self._last_rfid_tag != "None"): 
            self.get_logger().info('RFID did change')
            self._new_rfid_tag = self._last_rfid_tag
            return True
        else:
            return False 

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_handle_lock: #locks the next block for another thread, just one thread can access it
            if (self._goal_handle is not None and self._goal_handle.is_active):
                self.get_logger().info('Replace active goal with new goal.')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelling move')
        return CancelResponse.ACCEPT
    """
    Method to controll movement of the robot 
    Using an object of RobotMove to calculate velocities based on the subscribed data
    publishing the calculated velocity and sending out feedback in a rate of 20Hz
    terminates when either vel is None, an new RFID is detected or an ArUco marker is detected
    Finally publishes an empty velocity to stop the current movement 
    """
    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing move')
        mover = RobotMove(goal_handle.request.target_velocity[1])         
        vel = mover.follow_line(self._dist_to_line, goal_handle.request.target_velocity[0])       
        while(goal_handle.is_active and not goal_handle.is_cancel_requested and vel is not None 
                and  self.rfid_changed() == False and self.dist_to_robot == -1.0):
            vel = mover.follow_line(self._dist_to_line, goal_handle.request.target_velocity[0])
            self._publish_velocity(vel)
            self._publish_feedback(goal_handle, vel)
            time.sleep(0.05)
            
        self._publish_velocity(None)
        return self._determine_action_result(goal_handle)

    def _publish_velocity(self, vel):
        velocity_msg = Twist() 
        if(vel is not None):
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0]) 
            self._cmd_pub.publish(velocity_msg)
        else:
            velocity_msg.angular.z = 0.0
            velocity_msg.linear.x = 0.0 
            self._cmd_pub.publish(velocity_msg)    

    def _publish_feedback(self, goal_handle, vel):
        if(vel is not None):
            feedback_msg = Move.Feedback()
            velocity_msg = Twist()            
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0])
            feedback_msg.current_velocity = velocity_msg
            goal_handle.publish_feedback(feedback_msg)
    """
    Method to determine result
    Returns based on the current state of goal_handle and the subscribed data whether the result is
    aborted, canelled, has detected a robot or reached an RDIF-Tag
    """
    def _determine_action_result(self, goal_handle):
        result = Move.Result()
        if goal_handle.is_active and self._last_rfid_tag!="None": 
            self.get_logger().info('Move succeeded')
            self.did_RFID_change=False
            goal_handle.succeed()
            result.result = 'RFID_reached'
        elif goal_handle.is_active and self.dist_to_robot != -1.0:
            self.get_logger().info('Move succeeded')
            goal_handle.succeed()
            result.result = 'Robot_detected'
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Move was canceled')
            result.result = 'Canceled'
        else:
            if goal_handle.is_active:
                goal_handle.abort()
                self.get_logger().info('Move was aborted')
                result.result = 'Aborted'  
        self._publish_velocity(None) #affirms motorstop at the end of every action
        return result

def main():
    rclpy.init()
    try:
        robot_mover_executor = MultiThreadedExecutor(4)
        move_server = MoveServer()
        rclpy.spin(node=move_server, executor=robot_mover_executor)

    finally:
        move_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
