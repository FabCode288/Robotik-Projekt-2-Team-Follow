import time
from mfrc522 import SimpleMFRC522

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

"""
Creating a new minimal publisher to publish the value of the RFID sensor with a constant frequency
"""

class Minimal_RFID_Publisher(Node):

    def __init__(self):
        super().__init__('rfid_publisher')
        self.publisher_ = self.create_publisher(String, 'rfid_topic', 10) #Creating the publisher
        timer_period = 2.0  # pulishs Data String with 5Hz
        self.timer = self.create_timer(timer_period, self.timer_callback) #Creating the timer for a current frequency
        # self.rfid_id #The id of the rfid-chip
        # self.rfid_data #The data of the rfid-chip
        self.reader = SimpleMFRC522()

        self.i = 0
        self.chip = 1

    def timer_callback(self):
        """
        This Method is called after every timer period 
        The RFID reader reads the value and stores it
        If the reader reads a value, then the publisher publishs the data of the RFID sensor 
        If the reader does not read an value nothing happens
        """
        try:
            self.rfid_id, self.rfid_data = self.reader.read() #Reading the id and the data that is meassured by the rfid-chip
        except:
            self.rfid_id, self.rfid_data = None  #If reading the rfid failed the id and data is set to None
        if(self.rfid_id is not None and self.rfid_data is not None):
            msg = String()
            msg.data = self.rfid_data
            self.publisher_.publish(msg) #publishing the data
            self.get_logger().info('Publishing: "%s"' % msg.data)
        # if self.i<5:
        #     msg = String()
        #     msg.data = "None"
        #     self.publisher_.publish(msg)
        #     self.i += 1 
        #     print("send: None")
        #     time.sleep(2) 
        # else:
        #     if self.chip == 1 :
        #         msg = String()
        #         msg.data = "rfid1"
        #         end = time.time() + 8
        #         while(time.time()<end):
        #             self.publisher_.publish(msg)
        #             time.sleep(0.2)
        #             print("send: rfid1")
        #         self.chip = 2

        #     else:
        #         msg = String()
        #         msg.data = "rfid2"
        #         end = time.time() + 8
        #         while(time.time()<end):
        #             self.publisher_.publish(msg)
        #             time.sleep(0.2)
        #             print("send: rfid2")
        #         self.chip = 1
        #     time.sleep(12) 
        #     self.i = 0

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Minimal_RFID_Publisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()