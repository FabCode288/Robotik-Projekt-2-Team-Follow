#Start Befehl
#ros2 run ro36_simple_mover_server Ro36SimpleMoverServer
#Send empty Msg
#ros2 action send_goal -f /go_to ro36_interfaces/action/GoTo "{}"
#Send full Msg
#ros2 action send_goal -f /go_to ro36_interfaces/action/GoTo "{pose: {x: 4.0, y: 4.0, theta: 0.0}}"
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

from ro36_interfaces.action import Turn
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turn_server.robot_turn import RobotTurn

class TurnServer(Node):

    def __init__(self):
        super().__init__('turn_action_server')
        self._last_pose_x = 0
        self._last_pose_y = 0
        self._last_pose_roll = 0
        self._last_pose_pitch = 0
        self._last_pose_theta = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self._goal_handle = None

        self._action_server = ActionServer(
            self,
            Turn,
            'turn',
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
        #self.get_logger().info('Received odom_callback.')


    def _goal_callback(self, goal_request):
        self.get_logger().info('Received goal request with target turn')
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._goal_handle_lock: #locks the next block for another thread, just one thread can access it
            if (self._goal_handle is not None and self._goal_handle.is_active):
                self.get_logger().info('Replace active goal with new goal.')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelling turn')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing turn')
        mover = RobotTurn(self._last_pose_theta, goal_handle.request.target_velocity)
        #mover.set_target_pose(goal_handle.request.pose.x, goal_handle.request.pose.y, goal_handle.request.pose.theta)
        vel = mover.turn(self._last_pose_theta) 
        self.get_logger().info("Velocity1: {}, {}".format(vel[0], vel[1]))
        i=10 #dummy
        while(goal_handle.is_active and not goal_handle.is_cancel_requested and vel is not None and i>0 #dummy):
            vel = mover.turn(self._last_pose_theta) 
            self._publish_velocity(vel)
            self._publish_feedback(goal_handle, vel)
            time.sleep(0.05)
            i=i-1#dummy
        self._publish_velocity(None)
        return self._determine_action_result(goal_handle, vel)

    def _publish_velocity(self, vel):
        if(vel is not None):
            velocity_msg = Twist()
            self.get_logger().info("Velocity: {}, {}".format(vel[0], vel[1]))
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0]) 
            self._cmd_pub.publish(velocity_msg)


    def _publish_feedback(self, goal_handle, vel):
        feedback_msg = Turn.Feedback()
        velocity_msg = Twist()            
        velocity_msg.angular.z = float(vel[1])
        velocity_msg.linear.x = float(vel[0]) 
        feedback_msg.current_velocity = velocity_msg
        goal_handle.publish_feedback(feedback_msg)

    def _determine_action_result(self, goal_handle, vel):
        result = Turn.Result()
        if True:
            self.get_logger().info('Turn succeeded')
            goal_handle.succeed()
            result.result = "Turn_succesfull"
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Turn was canceled')
            result.result = "Canceled"
        else:
            if goal_handle.is_active:
                goal_handle.abort()
                self.get_logger().info('Turn was aborted')
                result.result = "Aborted"
        return result


def main():
    rclpy.init()
    try:
        robot_mover_executor = MultiThreadedExecutor(2)
        turn_server = TurnServer()
        rclpy.spin(node=turn_server, executor=robot_mover_executor)
    finally:
        turn_server.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
