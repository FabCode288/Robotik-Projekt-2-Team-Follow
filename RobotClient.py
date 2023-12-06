import rclpy
import string
import time
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import Image

from ro36_interfaces.action import Move
from ro36_interfaces.action import Follow
from ro36_interfaces.action import Turn

class SimpleRobotMoverClient(Node):

    def __init__(self):
        
        self.target_velocity= 0.1
        self.target_distance= 0.1
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

        self._rfid_sub = self.create_subscription(
            string,
            'rfid_topic',
            self._rfid_callback,
            10)
        self._camera_sub = self.create_subscription(
            Image,
            'camera_topic',
            self.image_callback,
            10)
        
    def _rfid_callback(self,msg):
            self._last_rfid_data = msg.data      
            #self.get_logger().info('Received rfid_callback.')  
            #   
    def _rfid_reached(self):   
        if (self._last_rfid_data == 1): 
            return True
        else:
            return False

    def _image_callback():
        pass

    def robot_detected():
        pass

    def send_goal_move(self):

        goal_msg = Move.Goal()
        goal_msg.float32 = self.target_velocity

        self._action_client_move.wait_for_server()
        self._send_goal_future = self._action_client_move.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_goal_follow(self):

        goal_msg = Follow.Goal()
        goal_msg.float32 = self.target_distance

        self._action_client_follow.wait_for_server()
        self._send_goal_future = self._action_client_follow.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_goal_turn(self):

        goal_msg = Turn.Goal()
        goal_msg.float32 = self.target_velocity/2

        self._action_client_turn.wait_for_server()
        self._send_goal_future = self._action_client_turn.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args) 
    client_logic = ClientLogic()
    client_node = SimpleRobotMoverClient()
    while():        
         match client_logic.get_next_move_order(client_node.last_result):
            case None:
                break
            case "turn":
                client_node._action_client_turn.send_goal_turn()
            case "move":
                cient_node._action_client_move.send_goal_move()
            case "follow":
                client_node._action_client_follow.send_goal_follow()
        time.sleep(0.1)
        
    rclpy.shutdown()
if __name__ == '__main__':
    main()
