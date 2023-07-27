import os
import rclpy
import numpy as np
import serial
import time
import sys
import glob

from rclpy.node import Node
from std_msgs.msg import Int16

class depthMeasurement(Node):

    def __init__(self):
        super().__init__('depth_measurement')

        #Declare node parameters
        self.declare_parameter('port', '') # Depth sensor serial port name

        # #Connect to serial port
        # portName = self.get_parameter('port').get_parameter_value().string_value
        # if portName == '':  # No user defined port
        #     ports = self.getSerialPorts()           # Get all available serial ports
        #     self.ser = self.connectToSerial(ports)  # Connect to the first port
        # else:               # With user defined port 
        #     self.ser = serial.Serial('/dev/' + portName, 57600, timeout=1.0) 

        #Published topics
        self.publisher_insertion = self.create_publisher(Int16, '/arduino/depth', 10)
        self.timer_depth = self.create_timer(1.0, self.timer_callback)

    def timer_callback (self):
        # self.ser.reset_input_buffer()
        # self.ser.write(b'h')
        # time.sleep(0.001)
        # data = self.ser.read(3)
        # line = list(data)   # read a '\n' terminated line
        # if len(line):
        #     decoded_bytes = int(data.decode('utf-8'))
        #     self.get_logger().debug('Depth in mm = %i'  %(decoded_bytes))
        #     msg = Int16()
        #     msg.data = int(decoded_bytes)
        #     self.publisher_insertion.publish(msg)
        decoded_bytes = int(0)
        self.get_logger().debug('Depth in mm = %i'  %(decoded_bytes))
        msg = Int16()
        msg.data = int(decoded_bytes)
        self.publisher_insertion.publish(msg)
    # Connect to the first available port with 1 second timeout
    def connectToSerial(self, ports):
        if len(ports) > 0:
            ser = serial.Serial(ports[0], baudrate=57600, timeout=1.0)
            self.get_logger().info('Connected to %s' %ser.name)
            return ser
        else:
            self.get_logger().info('No serial ports available')
            return None

    # Find available serial ports
    def getSerialPorts(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        available_ports = []
        for port in ports:
            try:
                s = serial.Serial(port, timeout=1)
                s.close()
                available_ports.append(port)
                self.get_logger().info('Available port: %s' %port)
            except (OSError, serial.SerialException):
                pass
        return available_ports
    
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
