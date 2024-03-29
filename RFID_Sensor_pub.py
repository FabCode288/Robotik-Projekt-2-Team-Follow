import rclpy

from rclpy.node import Node
from mfrc522 import SimpleMFRC522
from std_msgs.msg import String

"""
Creating a new minimal publisher to publish the value of the RFID sensor with a constant frequency
"""

class Minimal_RFID_Publisher(Node):

    def __init__(self):
        super().__init__('rfid_publisher')
        self.publisher_ = self.create_publisher(String, 'rfid_topic', 10) #Creating the publisher
        timer_period = 0.05 #pulishs Data String with 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback) #Creating the timer for a current frequency
        self.reader = SimpleMFRC522()

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
            self.publisher_.publish(msg) #publishing the data of the rfid chip

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Minimal_RFID_Publisher()
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    minimal_publisher.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
