import time
from mfrc522 import SimpleMFRC522

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Minimal_RFID_Publisher(Node):

    def __init__(self):
        super().__init__('rfid_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.2  # pulishs Data String with 5Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.rfid_id #The id of the rfid-chip
        self.rfid_data #The data of the rfid-chip

    def timer_callback(self):
        try:
            self.rfid_id, self.rfid_data = reader.read() #Reading the rfid-chip
        except:
            self.rfid_id, self.rfid_data = None  #If reading the rfid failed the id and data is set to None
        if(self.rfid_id is not None and self.rfid_data is not None):
            msg = String()
            msg.data = self.rfid_data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

def main(args=None):
    rclpy.init(args=args)
    reader = SimpleMFRC522()

    minimal_publisher = Minimal_RFID_Publisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

