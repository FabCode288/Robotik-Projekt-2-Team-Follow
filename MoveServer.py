import rclpy
import math
import time
import threading
from std_msgs.msg import String

from rclpy.action import ActionServer
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32

from ro36_interfaces.action import Move
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from move_server.robot_move import RobotMove

class MoveServer(Node):

    def __init__(self):
        super().__init__('move_action_server')
        self._last_rfid_tag = "None"
        self._new_rfid_tag = "None"
        self._dist_to_line = 0
        self.dist_to_robot=0
        


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
        self.get_logger().info('Read RFID im Callback: ' + format(msg.data))
       
    def _line_dist_callback(self, msg):
        self._dist_to_line = msg.data   
        self.get_logger().info('Distance to Line: ' + format(msg.data))

    def _robot_dist_callback(self, msg):
        self.dist_to_robot = msg.data

    def _goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
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

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing move')
        mover = RobotMove()         
        vel = mover.follow_line(self._dist_to_line, goal_handle.request.target)       
        while(goal_handle.is_active and not goal_handle.is_cancel_requested and vel is not None 
                and  self.rfid_changed() == False and self.dist_to_robot!=-1.0):
            vel = mover.follow_line(self._dist_to_line, 0.05)
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

    def _determine_action_result(self, goal_handle):
        self.get_logger().info('determine_action_result aufgerufen')

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
        return result


def main():
    rclpy.init()
    try:
        robot_mover_executor = MultiThreadedExecutor(3)
        move_server = MoveServer()
        rclpy.spin(node=move_server, executor=robot_mover_executor)
    finally:
        move_server.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
