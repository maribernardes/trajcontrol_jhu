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

from trajcontrol_interfaces.srv import GetJacobian
from ament_index_python.packages import get_package_share_directory
from .utils import get_angles

#########################################################################
#
# Estimator Node
#
# Description:
# This node gets the delta in needle_base and in needle_tip to estimate 
# the model Jacobian.
#
# Service server:    
# '/get_jacobian'  (trajcontrol_interfaces.srv.GetJacobian) - robot frame
#
#########################################################################

# This version only updates when get_jacobian service is requested
# Parameterized in insertion length

class Estimator(Node):

    def __init__(self):
        super().__init__('estimator')

        #Declare node parameters
        self.declare_parameter('alpha', 0.65)   # Jacobian update parameter
        self.declare_parameter('save_J', False) # Save Jacobian matrix in file

#### Stored variables ###################################################

        self.update = False
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.save_J = self.get_parameter('save_J').get_parameter_value().bool_value

        self.X = np.empty(shape=[3,0])                  # Current needle base pose X = [x_robot, y_needle_depth, z_robot]
        self.Z = np.empty(shape=[5,0])                  # Current needle tip pose Z = [x_tip, y_tip, z_tip, yaw, pitch] 
        self.Xant = np.empty(shape=[3,0])               # Previous X 
        self.Zant = np.empty(shape=[5,0])               # Previous Z 
      
#### Interface initialization ###################################################
        
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Load initial Jacobian from file
        try:
            trajcontrol_share_directory = get_package_share_directory('trajcontrol')
            self.get_logger().info('Loading initial Jacobian from %s/files' %(trajcontrol_share_directory))
            self.J  = np.array(loadtxt(os.path.join(trajcontrol_share_directory,'files','jacobian.csv'), delimiter=','))
            self.get_logger().info('Initial Jacobian transform\n J = %s' %(self.J))
        except IOError:
            self.get_logger().info('Could not find jacobian.csv file')
        if (self.save_J == True):
            self.get_logger().info('This trial will overwrite initial Jacobian file')
        else:
            self.get_logger().info('This trial will NOT overwrite initial Jacobian file')

#### Service server ###################################################

        # Service server to return jacobian
        self.service_server_jacobian = self.create_service(GetJacobian, '/estimator/get_jacobian', self.get_jacobian_callback)
        self.get_logger().info('Service /get_jacobian is now available')
        
#### Service server functions ###################################################

    # Send requested Jacobian matrix
    def get_jacobian_callback(self, request, response):
        # Update X
        base = request.base
        self.X = np.array([[base.x,base.y, base.z]]).T
        if (self.Xant.size == 0):
            self.Xant = self.X
        # Update Z
        tip = request.tip # Obs: request.tip is geometry_msgs/Pose, but the orientation was defines as qw = angle_horiz qx = angle_vert
        #TODO: Create better message - This is very bad practice
        self.Z = np.array([[tip.position.x, tip.position.y, tip.position.z, tip.orientation.w, tip.orientation.x]]).T
        if (self.Zant.size == 0):
            self.Zant = self.Z
        response.image_matrix = self.update_jacobian()
        return response

    # Update Jacobian from current base inputs and tip pose
    def update_jacobian(self):
        # Calculate delta
        deltaX = (self.X - self.Xant)
        deltaZ = (self.Z - self.Zant)
        # Update Jacobian
        self.J = self.J + self.alpha*np.outer((deltaZ-np.matmul(self.J, deltaX))/(np.matmul(np.transpose(deltaX), deltaX)+1e-9), deltaX)
        # Save previous values for next estimation
        self.Zant = self.Z
        self.Xant = self.X
        # Save updated Jacobian in file
        if (self.save_J == True):
            trajcontrol_share_directory = get_package_share_directory('trajcontrol')
            try:
                savetxt(os.path.join(trajcontrol_share_directory,'files','jacobian.csv'), asarray(self.J), delimiter=',')
                self.get_logger().info('Save Jacobian transform\n J = %s' %(self.J))
            except IOError:
                self.get_logger().error('Could NOT save Jacobian transform')
        else:
            self.get_logger().info('Jc = %s' %(self.J))
        J_image_matrix = CvBridge().cv2_to_imgmsg(self.J)
        J_image_matrix.header.stamp = self.get_clock().now().to_msg()
        return J_image_matrix

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