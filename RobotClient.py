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
        self._last_result_move = "Start"
        self._last_result_turn = "Start"
        self._last_result_follow = "Start"
        self.target_velocity_linear= 1.0
        self.target_velocity_angular = 1.0
        self.target_velocity = [self.target_velocity_linear,self.target_velocity_angular]
        self.target_distance= 0.1
        self.client_logic =ClientLogic()
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

  

    def send_goal_move(self):

        goal_msg = Move.Goal()
        goal_msg.target_velocity = self.target_velocity

        self._action_client_move.wait_for_server()        
        self._send_goal_future = self._action_client_move.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_move)
        self._send_goal_future.add_done_callback(self.goal_response_callback_move)
    
    def send_goal_follow(self):

        goal_msg = Follow.Goal()
        goal_msg.target_distance = self.target_distance

        self._action_client_follow.wait_for_server()
        self._send_goal_future = self._action_client_follow.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_follow)
        self._send_goal_future.add_done_callback(self.goal_response_callback_follow)
    
    def send_goal_turn(self):

        goal_msg = Turn.Goal()
        goal_msg.target_velocity = self.target_velocity_angular

        self._action_client_turn.wait_for_server()
        self._send_goal_future = self._action_client_turn.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_turn)
        self._send_goal_future.add_done_callback(self.goal_response_callback_turn)
    


    def goal_response_callback_move(self, future):        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future_move = goal_handle.get_result_async()
        self._get_result_future_move.add_done_callback(self.get_result_callback_move(future))

    def get_result_callback_move(self, future):
        self._last_result_move=future.result().get_result().result.result
        self.goal_trigger(self._last_result_move)
        self.get_logger().info('Result: ' + self._last_result_move)
        
        
    def feedback_callback_move(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_velocity.linear.x) + ' ' + format(feedback.current_velocity.angular.z))
        
    def goal_response_callback_turn(self, future):        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future_turn = goal_handle.get_result_async()
        self._get_result_future_turn.add_done_callback(self.get_result_callback_turn(future))

    def get_result_callback_turn(self, future):
        self._last_result_turn=future.result().get_result().result.result
        self.goal_trigger(self._last_result_turn)
        self.get_logger().info('Result: ' + self._last_result_turn)
        
        
    def feedback_callback_turn(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_velocity.linear.x) + ' ' + format(feedback.current_velocity.angular.z))

    def goal_response_callback_follow(self, future):        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future_follow = goal_handle.get_result_async()
        self._get_result_future_follow.add_done_callback(self.get_result_callback_follow(future))

    def get_result_callback_follow(self, future):
        self._last_result_follow=future.result().get_result().result.result
        self.goal_trigger(self._last_result_follow)
        self.get_logger().info('Result: ' + self._last_result_follow)
        
        
    def feedback_callback_follow(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_velocity.linear.x) + ' ' + format(feedback.current_velocity.angular.z))
    
    def goal_trigger(self,result):
        order = self.client_logic.get_next_client_order(result)
        match order:
            case "move":
                self.send_goal_move()
            case "turn":
                self.send_goal_turn()
            case "follow":
                self.send_goal_follow()
        
 

def main(args=None):
    rclpy.init(args=args) 
    try:        
        client_node = RobotClient()
        robot_client_executor= MultiThreadedExecutor(3)
        client_node.goal_trigger("Start")        
        rclpy.spin(node=client_node,executor=robot_client_executor)                  
    finally:
        client_node.destroy_node()   
        rclpy.shutdown()
if __name__ == '__main__':
    main()