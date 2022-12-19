import os
import rclpy
import math
import numpy as np
import quaternion

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge.core import CvBridge
from numpy import loadtxt, asarray, savetxt
from std_msgs.msg import Int8

#This version only publishes the Jacobian when there is a new /sensor/base
class Estimator(Node):

    def __init__(self):
        super().__init__('estimator')

        #Declare node parameters
        self.declare_parameter('alpha', 0.65)   #Jacobian update parameter
        self.declare_parameter('save_J', False) #Save Jacobian matrix in file

        #Topics from sensor processing node
        self.subscription_tip = self.create_subscription(PoseStamped, '/sensor/tip', self.tip_callback, 10)
        self.subscription_tip # prevent unused variable warning

        #Topics from robot node
        self.subscription_base = self.create_subscription(PoseStamped, '/sensor/base', self.base_callback, 10)
        self.subscription_base # prevent unused variable warning

        #Published topics (no timer - waits for SPACE key)
        self.publisher_jacobian = self.create_publisher(Image, '/needle/state/jacobian', 10)
        
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Load initial Jacobian from file
        try:
            self.J = np.array(loadtxt(os.path.join(os.getcwd(),'src','trajcontrol','files','jacobian.csv'), delimiter=','))
        except IOError:
            self.get_logger().info('Could not find initial jacobian.csv file')

        self.Z = np.empty(shape=[5,0])                  # Current needle tip pose Z = [x_tip, y_tip, z_tip, yaw, pitch] 
        self.X = np.empty(shape=[3,0])                  # Current needle base pose X = [x_robot, y_needle_depth, z_robot]
        self.Xant = np.empty(shape=[3,0])               # Previous X 
        self.Zant = np.empty(shape=[5,0])               # Previous Z 
        self.TX = self.get_clock().now().to_msg()       # Current X instant (time)
        self.TZ = self.get_clock().now().to_msg()       # Current Z instant (time)        
        self.TXant = self.TX                            # Previous X instant (time)
        self.TZant = self.TZ                            # Previous Z instant (time)

        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.save_J = self.get_parameter('save_J').get_parameter_value().bool_value
        if (self.save_J == True):
            self.get_logger().info('This trial will overwrite initial Jacobian file')
        else:
            self.get_logger().info('This trial will NOT overwrite initial Jacobian file')

    # Get current needle tip from sensor processing node
    # Z = [x_tip, y_tip, z_tip, yaw, pitch]  
    def tip_callback(self, msg_tip):
        # Get filtered sensor in robot frame   
        quat = np.array([msg_tip.pose.orientation.w, msg_tip.pose.orientation.x, msg_tip.pose.orientation.y, msg_tip.pose.orientation.z])
        angles = get_angles(quat)
        self.Z = np.array([[msg_tip.pose.position.x, msg_tip.pose.position.y, msg_tip.pose.position.z, angles[0], angles[1]]]).T
        self.TZ = msg_tip.header.stamp
        if (self.Zant.size == 0):
            self.Zant = self.Z
            self.TZant = self.TZ

    # Get current base from sensor processing node
    # X = [x_robot, y_needle_depth, z_robot]
    def base_callback(self, msg_base):
        # Get robot x and z from PoseStamped
        # From robot, get input X
        self.X = np.array([[msg_base.pose.position.x, msg_base.pose.position.y, msg_base.pose.position.z]]).T
        self.TX = msg_base.header.stamp
        if (self.Xant.size == 0):
            self.Xant = self.X
            self.TXant = self.TX
        self.update_jacobian()

    # Update Jacobian from current base inputs and tip pose
    def update_jacobian(self):
        
        deltaX = (self.X - self.Xant)
        deltaZ = (self.Z - self.Zant)
        
        # Update Jacobian
        self.J = self.J + self.alpha*np.outer((deltaZ-np.matmul(self.J, deltaX))/(np.matmul(np.transpose(deltaX), deltaX)+1e-9), deltaX)

        # Save previous values for next estimation
        self.Zant = self.Z
        self.TZant = self.TZ
        self.Xant = self.X
        self.TXant = self.TX

        # Save updated Jacobian in file
        if (self.save_J == True):
            self.get_logger().debug('Save Jacobian transform %s' %(self.J))
            savetxt(os.path.join(os.getcwd(),'src','trajcontrol','files','jacobian.csv'), asarray(self.J), delimiter=',')

        # Publish current Jacobian
        msg = CvBridge().cv2_to_imgmsg(self.J)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_jacobian.publish(msg)


########################################################################
### Auxiliar functions ###
########################################################################

# Function: get_angles(q)
# DO: Get needle angles in horizontal and vertical plane
# Inputs: 
#   q: quaternion (numpy array [qw, qx, qy, qz])
# Output:
#   angles: angle vector (numpy array [horiz, vert])
def get_angles(q):

    #Define rotation quaternion
    q_tf= np.quaternion(q[0], q[1], q[2], q[3])

    #Get needle current Z axis vector (needle insertion direction)
    u_z = np.quaternion(0, 0, 0, 1)
    v = q_tf*u_z*q_tf.conj()

    #Angles are components in x (horizontal) and z(vertical)
    horiz = math.atan2(v.x, -v.y)
    vert = math.atan2(v.z, math.sqrt(v.x**2+v.y**2))
    return np.array([horiz, vert])


########################################################################
def main(args=None):
    rclpy.init(args=args)

    estimator = Estimator()

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()