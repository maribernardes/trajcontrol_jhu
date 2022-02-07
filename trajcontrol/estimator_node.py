import rclpy
import message_filters
import math
import numpy as np
import quaternion

from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge.core import CvBridge
from numpy import linalg

class EstimatorNode(Node):

    def __init__(self):
        super().__init__('estimator_node')

        #Declare node parameters
        self.declare_parameter('alpha', 0.65) #Jacobian update parameter

        #Topics from UI node
        self.subscription_UI = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_UI  # prevent unused variable warning

        #Topics to be deleted after successful synchonization is confirmed
        self.subscription_test_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.subscription_robot_asynch, 10)
        self.subscription_test_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.subscription_sensor_asynch, 10)

        #Syncronized topics (robot and sensor nodes)
        self.subscription_robot = message_filters.Subscriber(self, PoseStamped, '/stage/state/needle_pose')
        self.subscription_sensor = message_filters.Subscriber(self, PoseArray, '/needle/state/current_shape')
        self.timeSync = message_filters.ApproximateTimeSynchronizer([self.subscription_robot, self.subscription_sensor], 10, 0.1)
        self.timeSync.registerCallback(self.sync_callback)

        #Published topics
        self.publisher_jacobian = self.create_publisher(Image, '/needle/state/jacobian', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_jacobian_callback)
        
        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Initialize Jacobian with estimated values from previous experiments
        # (Alternative: initialize with values from first two sets of sensor and robot data)
        self.J = np.array([(-0.3482, 0.1089, 0.0893,-0.1670, 0.1967, 0.0913, 0.1103),
                  ( 0.3594, 0.1332,-0.2593, 0.1975, 0.7322, 0.7989, 0.0794),
                  (-0.1714, 0.0723, 0.1597, 0.8766, 0.0610,-0.4968, 0.2415),
                  ( 0.0003, 0.0000,-0.0005, 0.0079, 0.0007,-0.0025, 0.0021),
                  (-0.0004,-0.0001, 0.0006,-0.0077,-0.0006, 0.0025,-0.0020),
                  (-0.0017,-0.0006, 0.0009, 0.0040, 0.0083, 0.0053,-0.0007),
                  (0, 0, 0, 0, 0, 0, 0)])

        self.Xant = np.zeros((7,1))
        self.Zant = np.zeros((7,1))
        self.TXant = self.get_clock().now().to_msg()
        self.TZant = self.get_clock().now().to_msg()

        self.i = 0

##############################################################################################################################
# EXTRA SUBSCRIPTIONS FOR TESTING PURPOSES - Delete after integration demo is successful
##############################################################################################################################
    # Test '/stage/state/needle_pose'
    def subscription_robot_asynch(self, msg):
        ##########################################
        # TODO: Delete this callback after demo is tested
        ##########################################

        # Get needle pose from PoseStamped
        needle = msg.pose
        
        # From robot/needle, get X
        X = np.array([[needle.position.x, needle.position.y, needle.position.z, needle.orientation.x, \
            needle.orientation.y, needle.orientation.z, needle.orientation.w]]).T
        
        self.get_logger().info('Listening robot - Sample #%i (asynch): X = %s in %s frame - Received from /stage/state/needle_pose' % (self.i, X.T, msg.header.frame_id))
        

    # Test '/needle/state/current_shape'
    def subscription_sensor_asynch(self, msg):
        ##########################################
        # TODO: Delete this callback after demo is tested
        ##########################################
        
        # Get needle shape from PoseArray
        shape = msg.poses
        
        # From shape, get Z
        N = len(shape)
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  #tip position
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])  #tip orientation

        Z = np.array([[tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]]]).T  #use np.array([[ ]]).T to get column vector

        self.get_logger().info('Listening shape - Sample #%i (asynch): Z = %s in %s frame - Received from /needle/state/current_shape' % (self.i, Z.T, msg.header.frame_id))

##############################################################################################################################

    # Get entry_point from UI
    def entry_point_callback(self, msg):
        ##########################################
        # TODO: Define transform from UI to robot frame
        ##########################################
        entry_point = msg.pose.position

        self.get_logger().info('Listening UI - Skin entry point: x=%f, y=%f, z=%f in %s frame'  % (entry_point.x, entry_point.y, \
            entry_point.z, msg.header.frame_id))        
    
    # Get current needle_pose from robot node
    # Perform estimator input X ("Prediction")
    # X = [x_robot, y_needle_depth, z_robot, q_needle_roll]
    # Get current needle shape from FBG sensor measurements
    # Perform estimator "correction"
    # Z = [x_tip, y_tip, z_tip, q_tip] (Obs: for q, roll=pitch)
    def sync_callback(self, msg_robot, msg_sensor):

        # Get needle pose from PoseStamped
        needle = msg_robot.pose
        TX = msg_robot.header.stamp
        
        ##########################################
        # TODO: Check quaternion order of elements
        ##########################################
        # From needle, get input X
        X = np.array([[needle.position.x, needle.position.y, needle.position.z, needle.orientation.x, \
            needle.orientation.y, needle.orientation.z, needle.orientation.w]]).T

        # Get needle shape from PoseArray
        shape = msg_sensor.poses
        TZ = msg_sensor.header.stamp
        
        # From shape, get measured Z
        N = len(shape)
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  #get tip
        if (msg_sensor.header.frame_id=='needle'):
          tip = needle2robot(tip)     #tranform from needle to robot frame
        ##########################################
        # TODO: Check use of orientation from shape instead of calculating myself
        ##########################################
        if (N==1):
            q = [0, 0, math.cos(math.pi/4), math.cos(math.pi/4)]
        else:
            ptip = needle2robot(np.array([shape[N-2].position.x, shape[N-2].position.y, shape[N-2].position.z])) #prior to tip
            forw = tip - ptip
            q = upforw2quat([0, 0, 1], forw)

        Z = np.array([[tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]]]).T  #use np.array([[ ]]).T to get column vector

        deltaTX = ((TX.sec*1e9 + TX.nanosec) - (self.TXant.sec*1e9 + self.TXant.nanosec))*1e-9
        deltaTZ = ((TZ.sec*1e9 + TZ.nanosec) - (self.TZant.sec*1e9 + self.TZant.nanosec))*1e-9
        
        deltaZ = (Z - self.Zant)/deltaTZ
        deltaX = (X - self.Xant)/deltaTX

        self.Zant = Z
        self.Xant = X
        self.TXant = TX
        self.TZant = TZ

        alpha = self.get_parameter('alpha').get_parameter_value().double_value
        if (self.i > 0): #Does nothing if first sample (no deltas)
            self.J = self.J + alpha*np.matmul(((deltaZ-np.matmul(self.J, deltaX))/(np.matmul(np.transpose(deltaX), deltaX)+1e-9)), np.transpose(deltaX))

        self.get_logger().info('Sample #%i: X = %s in robot frame' % (self.i, X.T))
        self.get_logger().info('Sample #%i: Z = %s in robot frame' % (self.i, Z.T))
        #self.get_logger().info('Sample #%i: J = \n%s' %  (self.i, self.J))
        self.i += 1

    # Publish current Jacobian matrix
    def timer_jacobian_callback(self):

        msg = CvBridge().cv2_to_imgmsg(self.J)
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_jacobian.publish(msg)
        #self.get_logger().info('Publish - Jacobian: %s' %  self.J)

########################################################################
### Auxiliar functions ###

# Function: needle2robot
# DO: Transform 3D point from needle frame to robot frame
# Input: point in needle frame (numpy-array [x, y, z])
# Output: point in robot frame (numpy-array [x, y, z])
def needle2robot(xn):

    #Define frame transformation
    rotx = np.quaternion(math.cos(-math.pi/4), math.sin(-math.pi/4),0,0)   # [cos(-90/2), sin(-90/2)*[1,0,0]]
    rotz = np.quaternion(math.cos(math.pi/2), 0, 0, math.sin(math.pi/2))   # [cos(180/2), sin(180/2)*[0,0,1]]
    rns = rotx*rotz
    ##########################################
    # TODO: Add x, y and z values in pns
    ##########################################    
    pns = np.quaternion(0, 0, 0, 0)                                        #[0, x, y, z] 

    #Build pure quaternion with point in needle frame
    pxn = np.quaternion(0, xn[0], xn[1], xn[2])
    
    #Transform to robot frame
    pxs = pns + rns*pxn*rns.conj()
    

    xs = np.array([ pxs.x, pxs.y, pxs.z ])
    return xs

# Function: upforw2quat
# DO:Get quaternion from up and forward vectors
# Input: up and forward vectors (3d float arrays)
# Output: quaternion (4d float array)
def upforw2quat(up, forw):
    left = np.cross(up, forw)
    up = np.cross(forw, left) # make up orthogonal

    # normalize vector just in case
    up = up/np.linalg.norm(up)
    forw = forw/np.linalg.norm(forw)
    left = left/np.linalg.norm(left)

    # build rotation matrix M
    m11 = left[0]
    m12 = left[1]
    m13 = left[2]
    m21 = up[0]
    m22 = up[1]
    m23 = up[2]
    m31 = forw[0]
    m32 = forw[1]
    m33 = forw[2]

    tr = m11+m22+m33 # trace of M
    if (tr>0):
        s = 2*math.sqrt(tr+1.0)
        q0 = 0.25*s
        q1 = (m32-m23)/s
        q2 = (m13-m31)/s
        q3 = (m21-m12)/s
    else:
        if ((m11>m22) and (m11>m33)):
            s = 2*math.sqrt(1.0+m11-m22-m33)
            q0 = (m32-m23)/s
            q1 = 0.25*s
            q2 = (m12+m21)/s
            q3 = (m13+m31)/s
        else:
            if (m22>m33):
                s = 2*math.sqrt(1.0+m22-m11-m33)
                q0 = (m13-m31)/s
                q1 = (m12+m21)/s
                q2 = 0.25*s
                q3 = (m23+m32)/s
            else:
                s = 2*math.sqrt(1.0+m33-m11-m22)
                q0 = (m21-m12)/s
                q1 = (m13+m31)/s
                q2 = (m23+m32)/s
                q3 = 0.25*s
    q = [q0, q1, q2, q3]
    return q/np.linalg.norm(q) # normalize q to return unit quaternion

########################################################################

def main(args=None):
    rclpy.init(args=args)

    estimator_node = EstimatorNode()

    rclpy.spin(estimator_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
