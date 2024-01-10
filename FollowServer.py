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

from ro36_interfaces.action import Follow
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from follow_server.robot_follow import RobotFollow

class FollowServer(Node):

    def __init__(self):
        super().__init__('follow_action_server')
        self._last_pose_x = 0
        self._last_pose_y = 0
        self._last_pose_roll = 0
        self._last_pose_pitch = 0
        self._last_pose_theta = 0

        self.dist_to_line = 0
        self.dist_to_robot = 0

        #sub camera
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

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
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

    def _odom_callback(self, msg):
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y
        self._last_pose_roll, self._last_pose_pitch, self._last_pose_theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w 
            ]
        )
        #self.get_logger().info('Received odom_callback.')

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
        mover = RobotFollow(goal_handle.request.distance, goal_handle.request.v)
        vel = mover.follow(self.dist_to_robot, self.dist_to_line) 
        self.get_logger().info("Velocity1: {}, {}".format(vel[0], vel[1])) #zu distance ändern
        i=0
        while(goal_handle.is_active and not goal_handle.is_cancel_requested and i<5):#if vel is None, wait for 5 Iterations before stopping, to compensate for lost frames
            vel = mover.follow(self.dist_to_robot, self.dist_to_line) 
            if vel is None:
                i += 1
            else:
                self._publish_velocity(vel)
                self._publish_calculated_feedback(goal_handle)
                i = 0
                time.sleepms(50)
                
        self._publish_velocity(None)
        return self._determine_action_result(goal_handle)

    def _publish_velocity(self, vel):
        if(vel is not None):
            velocity_msg = Twist()
            self.get_logger().info("Velocity: {}, {}".format(vel[0], vel[1]) )
            velocity_msg.angular.z = vel[1]
            velocity_msg.linear.x = vel[0] 
            self._cmd_pub.publish(velocity_msg)

    def _dist_to_robot(self, goal_handle):
        return 2    #calculate dist to robot

    def _publish_calculated_feedback(self, goal_handle):
        feedback_msg = Follow.Feedback()
        feedback_msg.dist_to_robot = self._dist_to_robot(goal_handle)
        goal_handle.publish_feedback(feedback_msg)

    def _determine_action_result(self, goal_handle):
        result = Follow.Result()
        if goal_handle.is_active and self._dist_to_robot(goal_handle) == None:
            self.get_logger().info('Follow succeeded') #erst nach wiederholtem aufruf successful
            goal_handle.succeed()
            result.reached = True
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Follow was canceled')
        else:
            if goal_handle.is_active:
                goal_handle.abort()
                self.get_logger().info('Follow was aborted')
        return result


def main():
    rclpy.init()
    robot_mover_executor = MultiThreadedExecutor(4)
    follow_server = FollowServer()
    try:
        rclpy.spin(node=follow_server, executor=robot_mover_executor)
    finally:
        follow_server.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
