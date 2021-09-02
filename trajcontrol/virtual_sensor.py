import rclpy
import numpy as np
import ament_index_python

from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from scipy.io import loadmat
from builtin_interfaces.msg import Time

class VirtualSensor(Node):

    def __init__(self):
        super().__init__('virtual_sensor')

        #Published topics
        self.publisher_shape = self.create_publisher(PoseArray, '/needle/state/shape', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Load data from matlab file
        package_path = str(ament_index_python.get_package_share_path('trajcontrol'))
        file_path = package_path + '/../../../../files/fbg_10.mat'
        #file_path = package_path + '/../../../../files/aurora_26.mat'
        trial_data = loadmat(file_path, mat_dtype=True)
        
        self.sensor = trial_data['sensor'][0]
        self.time_stamp = trial_data['time_stamp'][0]
        self.i=0
        
    # Publish current needle shape (PoseArray of 3D points)
    def timer_callback(self):
        
        # Use Aurora timestamp
        now = self.get_clock().now().to_msg()
        decimal = np.mod(self.time_stamp[self.i],1)
        now.nanosec = int(decimal*1e9)
        now.sec = int(self.time_stamp[self.i]-decimal)
    
        msg = PoseArray()
        msg.header.stamp = now
        msg.header.frame_id = "needle"

        # Populate message with X data from matlab file
        if (self.i < self.sensor.size):
            X = self.sensor[self.i]
            j = 0
            while (j < X.size/3):
                pose = Pose()
                pose.position.x = float(X[0][j])
                pose.position.y = float(X[1][j])
                pose.position.z = float(X[2][j])
                msg.poses.append(pose)
                j += 1
            self.i += 1

        self.publisher_shape.publish(msg)
        #self.get_logger().info('Publish - Pose Array %i = %s in %s frame' % (self.i, msg.poses, msg.header.frame_id))

def main(args=None):
    rclpy.init(args=args)

    virtual_sensor = VirtualSensor()

    rclpy.spin(virtual_sensor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()