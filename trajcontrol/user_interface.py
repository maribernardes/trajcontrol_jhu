import os
import rclpy
import numpy as np

import ros2_igtl_bridge.msg as OpenIGTL
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Point

class UserInterface(Node):

    def __init__(self):
        super().__init__('user_interface')

        #Topics from 3DSlicer (OpenIGTLink Bridge)
        self.subscription_bridge_point = self.create_subscription(OpenIGTL.Point, 'IGTL_POINT_IN', self.bridge_point_callback, 10)
        self.subscription_bridge_point # prevent unused variable warning
        
        # Published topics 
        timer_period = 1.0  # seconds
        self.timer_entry_point = self.create_timer(timer_period, self.timer_entry_point_callback)        
        self.timer_target = self.create_timer(timer_period, self.timer_target_callback)   
        self.publisher_entry = self.create_publisher(PointStamped, '/subject/state/skin_entry', 10)
        self.publisher_target = self.create_publisher(PointStamped, '/subject/state/target', 10)

        #Stored values
        self.entry_point = np.empty(shape=[0,3])
        self.target = np.empty(shape=[0,3])    

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

    # Get current robot pose
    def bridge_point_callback(self, msg_point):
        name = msg_point.name      
        self.get_logger().info('Message type - %s' %(name))
        if (name == "ENTRY_POINT-1"): # Name is adjusted in 3DSlicer module
            self.entry_point = np.array([msg_point.pointdata.x, msg_point.pointdata.y, msg_point.pointdata.z])
        elif (name == "TARGET_POINT-1"):
            self.target = np.array([msg_point.pointdata.x, msg_point.pointdata.y, msg_point.pointdata.z])
            
    # Publishes entry point
    def timer_entry_point_callback(self):
        # Publishes only after entry point is available
        if (self.entry_point.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "stage"
            msg.point = Point(x=self.entry_point[0], y=self.entry_point[1], z=self.entry_point[2])
            self.publisher_skin_entry.publish(msg)
            self.get_logger().info('Entry point = %s' %(self.entry_point))
            
    # Publishes target
    def timer_target_callback(self):
        # Publishes only after target is available
        if (self.target.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "stage"
            msg.point = Point(x=self.target[0], y=self.target[1], z=self.target[2])
            self.publisher_target.publish(msg) 
            self.get_logger().info('Target point = %s' %(self.target))
            

def main(args=None):
    rclpy.init(args=args)

    user_interface = UserInterface()

    rclpy.spin(user_interface)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    user_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()