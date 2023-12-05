import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ro36_interfaces.action import Move

class SimpleRobotMoverClient(Node):

    def __init__(self, type):
        self.type = type 
        super().__init__('pipe_client')
        try:
            if(type=="move"):
                self._action_client_move = ActionClient(
                    self,
                    Move, 
                    'move')
            elif(type=="follow"):
                self._action_client_follow = ActionClient(
                    self,
                    Follow, 
                    'follow')
            elif(type=="turn"):
                self._action_client_turn = ActionClient(
                    self,
                    Turn, 
                    'turn')
        except:
            pass

        self._rfid_sub = self.create_subscription(
            String,
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
            return true
        else:
            return false

    def _image_callback():
        pass

    def robot_detected():
        pass

    def send_goal_move(self, target_velocity):

        goal_msg = Move.Goal()
        goal_msg.float32 = target_velocity

        self._action_client_move.wait_for_server()
        self._send_goal_future = self._action_client_move.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_goal_follow(self, target_distance):

        goal_msg = Follow.Goal()
        goal_msg.float32 = target_velocity

        self._action_client_follow.wait_for_server()
        self._send_goal_future = self._action_client_follow.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_goal_turn(self, target_velocity):

        goal_msg = Turn.Goal()
        goal_msg.float32 = target_velocity

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
        rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

def main(args=None):
    rclpy.init(args=args)

    if(self.robot_detected()):
        action_client = SimpleRobotMoverClient("follow")
        future = action_client.send_goal_follow(10)
    elif(self._rfid_reached()):
        action_client = SimpleRobotMoverClient("turn")
        future = action_client.send_goal_turn(10)
    else:
        action_client = SimpleRobotMoverClient("move")
        future = action_client.send_goal_move(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()