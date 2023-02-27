import os
import rclpy
import numpy as np
import serial
import time

from rclpy.node import Node
from std_msgs.msg import Int8

RATE = 1.0

class depthMeasurement(Node):

    def __init__(self):
        super().__init__('depth_measurement')
        self.publisher_insertion = self.create_publisher(Int8, '/arduino/depth', 10)
        self.timer_depth = self.create_timer(RATE, self.timer_callback)

        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1.0) 

    def timer_callback (self):
        self.ser.reset_input_buffer()
        self.ser.write(b'h')
        time.sleep(0.001)
        data = self.ser.read(2)
        line = list(data)   # read a '\n' terminated line
        if len(line):
            decoded_bytes = int(data.decode("utf-8"))
            self.get_logger().info('depth in mm = %i'  %(decoded_bytes))
            msg = Int8()
            msg.data = int(decoded_bytes)
            self.publisher_insertion.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    depth = depthMeasurement()


    rclpy.spin(depth)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    depth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
