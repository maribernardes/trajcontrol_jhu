import os
import rclpy
import numpy as np
import serial
import time
import sys
import glob

from rclpy.node import Node
from std_msgs.msg import Int16, Int8
from trajcontrol.trajcontrol_demo_step import INSERTION_STEP

class virtualDepthMeasurement(Node):

    def __init__(self):
        super().__init__('virtual_depth_measurement')
        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning

        #Published topics
        self.publisher_insertion = self.create_publisher(Int16, '/arduino/depth', 10)
        self.timer_depth = self.create_timer(1.0, self.timer_callback)

        #Internal variable
        self.depth = None


    # Initialize and update insertion depth
    def keyboard_callback(self, msg):
        if (self.depth is None) and (msg.data == 32): # SPACE: initialize needle entry point and insertion depth
            self.depth = 0.0    
        elif (self.depth is not None) and ((msg.data == 32) or (msg.data == 50) or (msg.data == 52) or (msg.data == 54) or (msg.data == 56)):
            self.depth = self.depth - INSERTION_STEP

    # Publish insertion depth value
    def timer_callback (self):
        if (self.depth is not None):
            msg = Int16()
            msg.data = int(self.depth)
            self.publisher_insertion.publish(msg)
            self.get_logger().debug('Depth in mm = %i'  %(self.depth))

def main(args=None):
    rclpy.init(args=args)
    virtual_depth = virtualDepthMeasurement()
    rclpy.spin(virtual_depth)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_depth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
