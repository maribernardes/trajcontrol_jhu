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

from trajcontrol_interfaces.srv import GetPose, GetMatrix
from ament_index_python.packages import get_package_share_directory

#########################################################################
#
# Estimator Node
#
# Description:
# This node gets the delta in needle_base and in needle_tip to estimate 
# the model Jacobian.
# 
# Subscribes:   
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped)  - robot frame
# '/needle/state/tip'           (geometry_msgs.msg.PoseStamped)   - robot frame
#
# Service client:    
# '/get_needle_tip'             (trajcontrol_interfaces.srv.GetPose) - robot frame
#  OBS: request.name: "tip"
# '/get_needle_base'            (trajcontrol_interfaces.srv.GetPose) - robot frame
#  OBS: request.name: base"
#
# Service server:    
# '/get_jacobian'             (trajcontrol_interfaces.srv.GetMatrix) - robot frame
#  OBS: request.name: "jacobian"
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
            self.zFrameToRobot  = np.array(loadtxt(os.path.join(trajcontrol_share_directory,'files','jacobian.csv'), delimiter=','))
        except IOError:
            self.get_logger().info('Could not find zframe.csv file')
        if (self.save_J == True):
            self.get_logger().info('This trial will overwrite initial Jacobian file')
        else:
            self.get_logger().info('This trial will NOT overwrite initial Jacobian file')

#### Service client ###################################################

        # SmartNeedle interface service client
        # OBS: Node will not spin until services are available
        self.service_client_tip = self.create_client(GetPose, '/get_needle_tip')
        if not self.service_client_tip.service_is_ready():
            self.get_logger().warn('/get_needle_tip service server not available, waiting...')
        while not self.service_client_tip.wait_for_service(timeout_sec=5.0):            
            pass
        self.get_logger().warn('/get_needle_tip service available')

        self.service_client_base = self.create_client(GetPose, '/get_needle_base')
        if not self.service_client_base.service_is_ready():
            self.get_logger().warn('/get_needle_base service server not available, waiting...')
        while not self.service_client_base.wait_for_service(timeout_sec=5.0):            
            pass
        self.get_logger().warn('/get_needle_base service available')

#### Service server ###################################################

        # Service server to return jacobian
        self.service_server_jacobian = self.create_service(GetMatrix, '/get_jacobian', self.get_jacobian_callback)
        self.get_logger().info('Service /get_jacobian is now available')

#### Listening callbacks ###################################################

    # Get robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.X = np.array([robot.x, robot.y, robot.z])
        
#### Service client functions ###################################################

    # Send get_tip request 
    def get_tip(self):
        request = GetPose.Request()
        request.name = 'tip'
        future = self.service_client_tip.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('result tip = %s' %future)
        return future.result()

    # Send get_base reqiest 
    def get_base(self):
        request = GetPose.Request()
        request.name = 'base'
        future = self.service_client_base.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('result base = %s' %future)
        return future.result()

#### Service server functions ###################################################

    # Send requested Jacobian matrix
    def get_jacobian_callback(self, request, response):
        self.update = True
        matrix_name = request.name
        if (matrix_name == 'jacobian'):
            self.get_logger().info('Received %s request' %(matrix_name))
            # self.update_jacobian()
            # # response.image_matrix = CvBridge().cv2_to_imgmsg(self.J)  
            # response.image_matrix = 'J'   
        return response

    # Update Jacobian from current base inputs and tip pose
    def update_jacobian(self):
        # Update X
        base_pose = self.get_base()
        self.get_logger().info('Update X = %s' %base_pose)
        if base_pose.valid is True:
            self.X = np.array([[base_pose.x, base_pose.y, base_pose.z]]).T
            if (self.Xant.size == 0):
                self.Xant = self.X
        else:
            self.get_logger().info('Invalid base pose')
        # Update Z
        tip_pose = self.get_tip()
        self.get_logger().info('Update Z = %s' %tip_pose)
        if tip_pose.valid is True:
            quat = np.array([tip_pose.qw, tip_pose.qx, tip_pose.qy, tip_pose.qz])
            angles = get_angles(quat)
            self.Z = np.array([[tip_pose.x, tip_pose.y, tip_pose.z, angles[0], angles[1]]]).T
            if (self.Zant.size == 0):
                self.Zant = self.Z
        else:
            self.get_logger().info('Invalid tip pose')
        # Calculate delta
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
        self.get_logger().info('Jacobian OK')

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