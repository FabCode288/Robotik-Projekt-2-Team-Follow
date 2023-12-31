import launch
import launch_ros.actions
import sys

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='move_server',
            executable='MoveServer',
            name='move_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='turn_server',
            executable='TurnServer',
            name='turn_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='follow_server',
            executable='FollowServer',
            name='follow_server',
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=[sys.executable, '/home/vinc/ros2_ws/src/rfid_pub/rfid_pub/RFID_Sensor_pub.py'],
            name='rfid_pub',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='robot_client',
            executable='RobotClient',
            name='robot_client',
            output='screen'
        ),
    ])
