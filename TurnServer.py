import rclpy
import time
import threading

from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turn_server.tf import euler_from_quaternion
from ro36_interfaces.action import Turn
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turn_server.robot_turn import RobotTurn

"""
Turn server to controll movement of a robot to turn 180 degrees
Has subscriptions on the odometry data of the robot
Publishes an angular velocity until the rotation is complete
"""

class TurnServer(Node):

    def __init__(self):
        super().__init__('turn_action_server')
        self._last_pose_theta = 0
        self._goal_handle = None
        self._goal_handle_lock = threading.Lock()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

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

    def _odom_callback(self, msg):
        _, _, self._last_pose_theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w 
            ]
        )

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
    
    """
    Method controlling the movement of the robot when turning
    Using an object of RobotTurn to calculate the velocity based on the degree value
     
    """

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing turn')
        mover = RobotTurn(self._last_pose_theta, goal_handle.request.target_velocity) #Ceating an instance of RobotTurn
        vel = mover.turn(self._last_pose_theta) #Calculating the turn velocity once befor entering the loop
        self.get_logger().info("Velocity: {}, {}".format(vel[0], vel[1]))
        while (goal_handle.is_active and not goal_handle.is_cancel_requested and vel is not None): #active while an velocity is calculated
            vel = mover.turn(self._last_pose_theta) #Calculating the velocity deping on the current degree value
            self._publish_velocity(vel) #publishing the velocity
            self._publish_feedback(goal_handle, vel)
            time.sleep(0.1) #Working with a frequency of 10Hz
        self._publish_velocity(None)
        return self._determine_action_result(goal_handle, vel)

    def _publish_velocity(self, vel):
        velocity_msg = Twist() 
        if(vel is not None):
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0]) 
        else:
            velocity_msg.angular.z = float(0.0)
            velocity_msg.linear.x = float(0.0) 
        self._cmd_pub.publish(velocity_msg) 

    def _publish_feedback(self, goal_handle, vel):
        if(vel is not None):
            feedback_msg = Turn.Feedback()
            velocity_msg = Twist()            
            velocity_msg.angular.z = float(vel[1])
            velocity_msg.linear.x = float(vel[0]) 
            feedback_msg.current_velocity = velocity_msg
            goal_handle.publish_feedback(feedback_msg)

    """
    Method to determine result
    Returns based on the current state of goal_handle
    """

    def _determine_action_result(self, goal_handle, vel):
        result = Turn.Result()
        if goal_handle.is_active:
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
        self._publish_velocity(None) #affirms motorstop at the end of every action
        return result

def main():
    rclpy.init()
    robot_mover_executor = MultiThreadedExecutor(2)
    turn_server = TurnServer()
    try:
        rclpy.spin(node=turn_server, executor=robot_mover_executor)
    except KeyboardInterrupt:
        turn_server._publish_velocity(None)
    turn_server.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
