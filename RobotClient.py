import rclpy
import string
import time
from rclpy.action import ActionClient
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Image

from ro36_interfaces.action import Move
from ro36_interfaces.action import Follow
from ro36_interfaces.action import Turn
from robot_client.client_logic import ClientLogic

import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class RobotClient(Node):

    def __init__(self):
        self._last_result = "Start"
        self.target_velocity_linear= 1.0
        self.target_velocity_angular = 1.0
        self.target_velocity = [self.target_velocity_linear,self.target_velocity_angular]
        self.target_distance= 0.1
        super().__init__('pipe_client')       
        self._action_client_move = ActionClient(
            self,
            Move, 
            'move',
            callback_group=ReentrantCallbackGroup())   
        self._action_client_follow = ActionClient(
            self,
            Follow, 
            'follow',
            callback_group=ReentrantCallbackGroup())    
        self._action_client_turn = ActionClient(
            self,
            Turn, 
            'turn',
            callback_group=ReentrantCallbackGroup())       

    #     self._rfid_sub = self.create_subscription(
    #         string,
    #         'rfid_topic',
    #         self._rfid_callback,
    #         10)
    #     self._camera_sub = self.create_subscription(
    #         Image,
    #         'camera_topic',
    #         self.image_callback,
    #         10)
        
    # def _rfid_callback(self,msg):
    #         self._last_rfid_data = msg.data      
    #         #self.get_logger().info('Received rfid_callback.')  
    #         #   
    # def _rfid_reached(self):   
    #     if (self._last_rfid_data == 1): 
    #         return True
    #     else:
    #         return False

    # def _image_callback():
    #     pass

    # def robot_detected():
    #     pass

    def send_goal_move(self):

        goal_msg = Move.Goal()
        goal_msg.target_velocity = self.target_velocity

        self._action_client_move.wait_for_server()        
        self._send_goal_future = self._action_client_move.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_goal_follow(self):

        goal_msg = Follow.Goal()
        goal_msg.target_distance = self.target_distance

        self._action_client_follow.wait_for_server()
        self._send_goal_future = self._action_client_follow.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_goal_turn(self):

        goal_msg = Turn.Goal()
        goal_msg.target_velocity = self.target_velocity_angular

        self._action_client_turn.wait_for_server()
        self._send_goal_future = self._action_client_turn.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    


    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback(future))

    def get_result_callback(self, future):
        self._last_result = future.result().get_result().result.result
        self.get_logger().info('Result: {0}'.format(future.result().get_result().result.result))
        
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_velocity.linear.x) + ' ' + format(feedback.current_velocity.angular.z))
 

def main(args=None):
    rclpy.init(args=args) 
    try:
        client_logic = ClientLogic()
        client_node = RobotClient()
        robot_client_executor= MultiThreadedExecutor(5)        
        order = None       
        while(True):
            last_order = order
            order=client_logic.get_next_client_order(client_node._last_result)   
            if last_order!=order:            
                match order:
                    case None:           
                        print("break")
                        break
                    case "turn":
                        client_node.send_goal_turn()                 
                        print("turn")              
                    case "move":
                        client_node.send_goal_move()                
                        print("move")               
                    case "follow":
                        client_node.send_goal_follow()                
                        print("follow")            
            rclpy.spin_once(node=client_node,executor=robot_client_executor)
        client_node.destroy_node()    
    finally:
        client_node.destroy_node()   
        rclpy.shutdown()
if __name__ == '__main__':
    main()