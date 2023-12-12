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

from ro36_interfaces.action import Move
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from move_server.robot_move import RobotMove

class MoveServer(Node):

    def __init__(self):
        super().__init__('move_action_server')
        self._last_pose_x = 0
        self._last_pose_y = 0
        self._last_pose_roll = 0
        self._last_pose_pitch = 0
        self._last_pose_theta = 0
        self._last_rfid_tag = "None"


        self.rfid_sub = self.create_subscription(
            String,
            'rfid_topic',
            self._rfid_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10)
        #  self.camera_sub = self.create_subscription(
        #     Bool,float32
        #     'camera_topic',
        #     self._camera_callback,
        #     10)

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

    def _odom_callback(self, msg):
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y
        self._last_pose_roll, 
        self._last_pose_pitch, 
        self._last_pose_theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w 
            ]
        )
     
    def _rfid_callback(self, msg):  
        self._last_rfid_tag = msg.data   
        self.get_logger().info('Read RFID: ' + format(msg.data))
       
    # def _camera_callback(self,msg1,msg2):
    #     self._last_rfid_data = self._current_rfid_data
    #     self._current_rfid_data = msg1.data      
    #     #self.get_logger().info('Received rfid_callback.')    


    def _goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

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
        vel = mover.get_movement_pipe(self._last_pose_pitch, self._last_pose_roll,
                                      goal_handle.request.target_velocity[0],
                                      goal_handle.request.target_velocity[1],self._last_rfid_tag,-1.0) #cam einfügen       
        while(goal_handle.is_active and not goal_handle.is_cancel_requested and vel is not None):
            vel = mover.get_movement_pipe(self._last_pose_pitch, self._last_pose_roll,
                                          goal_handle.request.target_velocity[0],
                                          goal_handle.request.target_velocity[1],self._last_rfid_tag,-1.0) #cam einfügen
            self._publish_velocity(vel)
            self._publish_feedback(goal_handle, vel)
            time.sleep(0.05)
            
        self._publish_velocity(None)
        return self._determine_action_result(goal_handle)

    def _publish_velocity(self, vel):
        velocity_msg = Twist() 
        if(vel is not None):
            self.get_logger().info("Velocity: {}, {}".format(vel[0], vel[1]) )
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0]) 
            self._cmd_pub.publish(velocity_msg)
        else:
            velocity_msg.angular.z = 0.0
            velocity_msg.linear.x = 0.0 
            self._cmd_pub.publish(velocity_msg)    

    # def _current_velocity(self, vel):
    #     if(vel is not None):
    #         velocity_msg = Twist()            
    #         velocity_msg.angular.z = vel[1]
    #         velocity_msg.linear.x = vel[0] 
    #         return velocity_msg
    #     else:
    #         return None
   
    # def _rfid_changed(self):
    #     if (self._current_rfid_data != "0" and self._last_rfid_tag != self._current_rfid_data ): 
    #         return True
    #     else:
    #         return False 

    def _publish_feedback(self, goal_handle, vel):
        if(vel is not None):
            feedback_msg = Move.Feedback()
            velocity_msg = Twist()            
            velocity_msg.angular.z = vel[1]
            velocity_msg.linear.x = vel[0] 
            feedback_msg.current_velocity = velocity_msg
            goal_handle.publish_feedback(feedback_msg)

    def _determine_action_result(self, goal_handle):
        result = Move.Result()
        if goal_handle.is_active and self._last_rfid_tag!="None": 
            self.get_logger().info('Move succeeded')
            self.did_RFID_change=False
            goal_handle.succeed()
            result.result = 'RFID_reached'
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