import rclpy
import threading

from rclpy.action import ActionClient
from rclpy.node import Node
from ro36_interfaces.action import Move
from ro36_interfaces.action import Follow
from ro36_interfaces.action import Turn
from robot_client.client_logic import ClientLogic
"""
Master unit to controll which action server is in controll of the robots movement
sends goals to a specific action server based on the result of the previous server
starts with move
"""
class RobotClient(Node):
    def __init__(self):
        self._last_result = "Start"
        self.target_velocity_linear= 0.1
        self.target_velocity_angular = 0.3
        self.target_velocity = [self.target_velocity_linear,self.target_velocity_angular]
        self.target_distance = 0.3
        self.client_logic = ClientLogic()

        self.lock_move = threading.Lock()
        self.lock_turn = threading.Lock()
        self.lock_follow = threading.Lock()

        super().__init__('pipe_client')       
        self._action_client_move = ActionClient(
            self,
            Move, 
            'move')   
        self._action_client_follow = ActionClient(
            self,
            Follow, 
            'follow')    
        self._action_client_turn = ActionClient(
            self,
            Turn, 
            'turn')       
"""
sends goal with the initialised target_velocity to the move action server 
"""
    def send_goal_move(self):

        goal_msg = Move.Goal()
        goal_msg.target_velocity = self.target_velocity

        self._action_client_move.wait_for_server()        
        self._send_goal_future_move = self._action_client_move.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future_move.add_done_callback(self.goal_response_callback)  
"""
sends goal with the initialised target_distance to the follow action server 
"""
    
    def send_goal_follow(self):

        goal_msg = Follow.Goal()
        goal_msg.target_distance = self.target_distance

        self._action_client_follow.wait_for_server()
        self._send_goal_future_follow = self._action_client_follow.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future_follow.add_done_callback(self.goal_response_callback)
"""
sends goal with the initialised target_velocity to the turn action server 
"""    
    def send_goal_turn(self):

        goal_msg = Turn.Goal()
        goal_msg.target_velocity = self.target_velocity_angular

        self._action_client_turn.wait_for_server()
        self._send_goal_future_turn = self._action_client_turn.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future_turn.add_done_callback(self.goal_response_callback)  

    def goal_response_callback(self, future):        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self._last_result = future.result().result.result
        self.goal_trigger(self._last_result)
        self.get_logger().info('Result: ' + self._last_result)    
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_velocity.linear.x) + ', ' + format(feedback.current_velocity.angular.z))
    
    def goal_trigger(self,result):
        order = self.client_logic.get_next_client_order(result)
        match order:
            case "move":
                with self.lock_move:
                    self.send_goal_move()
            case "turn":
                with self.lock_turn:
                    self.send_goal_turn()
            case "follow":
                with self.lock_follow:
                    self.send_goal_follow() 

def main(args=None):
    rclpy.init(args=args)
    client_node = RobotClient() 
    try:        
        client_node.goal_trigger("Start")        
        rclpy.spin(client_node)                  
    finally:
        client_node.destroy_node()   
        rclpy.shutdown()

if __name__ == '__main__':
    main()
