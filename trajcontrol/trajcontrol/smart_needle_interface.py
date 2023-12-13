import os
import rclpy
import numpy as np
import quaternion
import datetime

from rclpy.node import Node
from numpy import loadtxt
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from ros2_igtl_bridge.msg import PointArray, String
from smart_control_interfaces.srv import GetPose, GetPoint
from functools import partial

from numpy import loadtxt
from ament_index_python.packages import get_package_share_directory

#########################################################################
#
# SmartNeedle Interface Node
#
# Description:
# This node gets current needle shape, extracts the tip pose and converts
# it to robot frame. It also gets robot current position and extracts the
# needle base pose in needle frame.
# If use_slicer is set to True, it also pushes the needle shape to 3DSlicer
# throught OpenIGTLink bridge in zFrame coordinates
#
# Subscribes:   
# '/needle/state/current_shape' (geometry_msgs.msg.PoseArray)    - needle frame
# '/stage/state/guide_pose'     (geometry_msgs.msg.PointStamped) - robot frame
#
# Publishes:    
# '/stage/state/needle_pose'    (geometry_msgs.msg.PoseStamped)   - needle frame
# '/needle/state/skin_entry'    (geometry_msgs.msg.Point)         - needle frame
# 'IGTL_STRING_OUT'             (ros2_igtl_bridge.msg.String)     - zFrame
# 'IGTL_POINT_OUT'              (ros2_igtl_bridge.msg.PointArray) - zFrame
# 
# Service client:    
# '/planning/get_initial_point' (smart_control_interfaces.srv.GetPoint) - robot frame
# '/planning/get_skin_entry'    (smart_control_interfaces.srv.GetPoint) - robot frame
#
# Service server:    
# '/needle/get_tip'             (smart_control_interfaces.srv.GetPose) - robot frame
#
#################################################################################
class SmartNeedleInterface(Node):

#### Node initialization###################################################

    def __init__(self):
        super().__init__('smart_needle_interface')

        #Declare node parameters
        self.declare_parameter('use_slicer', True) # Push shape to OpenIGTLink bridge
        self.declare_parameter('needle_length', 200.0) # Needle length

    #### Stored variables ###################################################

        # Node parameters
        self.push_to_bridge = self.get_parameter('use_slicer').get_parameter_value().bool_value
        self.needle_length = self.get_parameter('needle_length').get_parameter_value().double_value

        self.zFrameToRobot = np.empty(shape=[0,7])      # zFrame to robot frame transform (from robot package)
        self.needleToRobot = np.empty(shape=[0,7])      # Needle to robot frame transform (initialized from initial_point)

        self.initial_point = np.empty(shape=[0,3])      # Robot position at begining of experiment (robot frame)
        self.skin_entry = np.empty(shape=[0,3])         # Skin entry at begining of experiment (robot frame)
        self.air_gap_needle = np.empty(shape=[0,3])     # Distance between needle guide and skin entry at begining of experiment (needle frame)
        
        self.stage = np.empty(shape=[0,3])              # Robot current position (robot frame)
        self.needle_pose = np.empty(shape=[0,7])        # Base pose (needle frame)
        self.tip = np.empty(shape=[0,7])                  # Tip pose (robot frame)

        # NeedleShape Bridge message
        self.shapecount = 0                         # Number of shape packace received
        self.shapeheader = None                     # Shape message header to push to 3D Slicer
        self.shapedata = None                       # Shape message data to push to 3D Slicer

    #### Subscribed topics ###################################################

        #Topics from smart needle node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.shape_callback,  10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PointStamped, '/stage/state/guide_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

    #### Published topics ###################################################

        # Needle tip (robot frame)
        # timer_period_tip = 0.3 # seconds
        # self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)        
        # self.publisher_tip = self.create_publisher(PoseStamped,'/needle/state/tip', 10)  #(robot frame)

        # Needle base (needle frame)
        timer_period_needle_pose = 0.3 # seconds
        self.timer_needle_pose = self.create_timer(timer_period_needle_pose, self.timer_needle_pose_callback)        
        self.publisher_needle_pose = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)  #(needle frame)
        
        # Air gap (needle frame)
        timer_period_air_gap_needle = 0.3 # seconds
        self.timer_air_gap_needle = self.create_timer(timer_period_air_gap_needle, self.timer_air_gap_needle_callback)        
        self.publisher_air_gap_needle = self.create_publisher(Point, '/needle/state/skin_entry', 10) #(needle frame)

        # Needle shape (zFrame)
        if self.push_to_bridge is True:
            timer_period_shape = 1.0 # seconds
            self.timer_shape = self.create_timer(timer_period_shape, self.timer_shape_callback)        
            self.publisher_shapeheader = self.create_publisher(String, 'IGTL_STRING_OUT', 10)
            self.publisher_shape = self.create_publisher(PointArray, 'IGTL_POINT_OUT', 10)

    #### Service client ###################################################

        # Planning services clients
        # OBS: Node will not spin until services are available
        self.initial_point_client = self.create_client(GetPoint, '/planning/get_initial_point')
        self.skin_entry_client = self.create_client(GetPoint, '/planning/get_skin_entry')
        if (not self.initial_point_client.service_is_ready()) or (not self.skin_entry_client.service_is_ready()):
            self.get_logger().info('Planning services not available, waiting...')
        while not self.initial_point_client.wait_for_service(timeout_sec=5.0):
            pass
        while not self.skin_entry_client.wait_for_service(timeout_sec=5.0):
            pass
        self.get_logger().info('Planning services available')
            
    #### Service server ###################################################

        # Service server to return planning points
        self.service_server_tip = None   # Activate only after self.tip is available

    #### Variables initialization ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.15f}".format(x)})

        # Load zFrameToRobot transform
        try:
            smart_template_share_directory = get_package_share_directory('smart_template')
            self.zFrameToRobot  = np.array(loadtxt(os.path.join(smart_template_share_directory,'files','zframe.csv'), delimiter=','))
        except IOError:
            self.get_logger().error('Could not find zframe.csv file')

        # Initialize planning points
        initial_point = self.get_initial_point()
        if initial_point.valid is True:
            self.initial_point = np.array([initial_point.x, initial_point.y, initial_point.z])
        else:
            self.get_logger().error('Invalid initial_point')
        skin_entry = self.get_skin_entry()
        if skin_entry.valid is True:
            self.skin_entry = np.array([skin_entry.x, skin_entry.y, skin_entry.z])
        else:
            self.get_logger().error('Invalid skin_entry point')
 
        # Set needleToRobot transform         
        q_tf1= np.quaternion(np.cos(np.deg2rad(-45)), np.sin(np.deg2rad(-45)), 0, 0)
        q_tf2= np.quaternion(np.cos(np.deg2rad(90)), 0, 0, np.sin(np.deg2rad(90)))
        q_tf = q_tf1*q_tf2  
        needle_base= np.array([self.initial_point[0], self.initial_point[1]-self.needle_length, self.initial_point[2]])  # needle base at experiment init
        self.needleToRobot = np.concatenate((needle_base, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        self.get_logger().info('skin_entry = %s' %self.skin_entry)
        self.get_logger().info('initial_point = %s' %self.initial_point)

        # Store air_gap in needle frame
        skin_entry_needle = pose_inv_transform(np.array([self.skin_entry[0], self.skin_entry[1], self.skin_entry[2], 1,0,0,0]), self.needleToRobot)[0:3]
        initial_point_needle = pose_inv_transform(np.array([self.initial_point[0], self.initial_point[1], self.initial_point[2], 1,0,0,0]), self.needleToRobot)[0:3]
        self.air_gap_needle = skin_entry_needle - initial_point_needle
        self.get_logger().info('/needle/state/skin_entry (air_gap) (needle frame) = %s' %self.air_gap_needle)

#################################################################################
#### Service client functions ###################################################

    # Request initial_point
    def get_initial_point(self):
        request = GetPoint.Request()
        future = self.initial_point_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    # Request skin_entry
    def get_skin_entry(self):
        request = GetPoint.Request()
        future = self.skin_entry_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

#### Service server functions ###################################################

    # Send requested tip pose in robot frame
    def get_tip_callback(self, request, response):
        if (self.tip.size == 0):
            response.valid = False
        else:
            response.valid = True
            response.x = self.tip[0]
            response.y = self.tip[1]
            response.z = self.tip[2]
            response.qw = self.tip[3]
            response.qx = self.tip[4]
            response.qy = self.tip[5]
            response.qz = self.tip[6]        
        return response    

#### Listening callbacks ###################################################

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.stage = np.array([robot.x, robot.y, robot.z])
        # Store current needle base pose (in needle frame)
        if (self.needleToRobot.size != 0):
            # Needle orientation in robot frame
            needle_q = self.needleToRobot[3:7]  
            # Remember to substract needle length to get the point at the needle BASE not at the guide                                    
            needle_pose_robot = np.array([self.stage[0], self.stage[1]-self.needle_length, self.stage[2], needle_q[0], needle_q[1], needle_q[2], needle_q[3]])
            self.needle_pose = pose_inv_transform(needle_pose_robot, self.needleToRobot)  # needle base in needle frame

    # Get current sensor measurements
    def shape_callback(self, msg_sensor):
        # From shape, get tip (last point)
        self.shapecount += 1
        shape = msg_sensor.poses      
        N = len(shape) 
        # Tip is last point in shape
        p_tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z]) 
        q_tip = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])
        self.get_logger().info('Tip q (needle) = %s' %q_tip)
        tip_needle = np.array([p_tip[0], p_tip[1], p_tip[2], q_tip[0], q_tip[1], q_tip[2], q_tip[3]]) # needle tip in needle frame
        # Transform from needle to robot frame
        if (self.needleToRobot.size != 0): 
            self.tip = pose_transform(tip_needle, self.needleToRobot)             # needle tip in robot frametimer_skin_entry_needle_callback
            if self.service_server_tip is None:
                self.service_server_tip = self.create_service(GetPose, '/needle/get_tip', self.get_tip_callback)
                self.get_logger().info('/needle/get_tip service is available')
            # Transform from needle to zFrame (to 3DSlicer)
            if self.push_to_bridge is True:
                # Build shape message to push to 3D Slicer
                frame_id = 'zFrame'
                timestamp = msg_sensor.header.stamp
                # Convert timestamp to a readable format
                now = datetime.datetime.now()
                timestamp_duration = datetime.timedelta(seconds=timestamp.nanosec / 1e9)
                duration_since_epoch = now - timestamp_duration
                # Get the time_t object from the datetime
                time_t_object = datetime.datetime.fromtimestamp(duration_since_epoch.timestamp())
                # Format the timestamp with seconds and milliseconds
                formatted_timestamp = time_t_object.strftime('%Y-%m-%d %H:%M:%S') + '.{:03d}'.format(int(timestamp.nanosec % 1e6 / 1e3))
                self.shapeheader = formatted_timestamp + ';' +str(self.shapecount) + ';'+ str(N) + ';' + frame_id
                # Get shape data points
                self.shapedata = []
                for pose in msg_sensor.poses:        
                    # Get next point and transform to zFrame
                    point_needle = np.array([pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])  
                    point_robot = pose_transform(point_needle, self.needleToRobot)
                    point_zFrame = pose_inv_transform(point_robot, self.zFrameToRobot)
                    # Save it in point array
                    point = Point()
                    point.x = point_zFrame[0]
                    point.y = point_zFrame[1]
                    point.z = point_zFrame[2]
                    self.shapedata.append(point)

#### Publishing callbacks ###################################################

    # Publishes needle shape (robot frame) to IGTLink bridge
    def timer_shape_callback(self):
        if (self.shapedata is not None) and (self.shapeheader is not None):
            string_msg = String()
            string_msg.name = 'NeedleShapeHeader'
            string_msg.data = self.shapeheader
            pointarray_msg = PointArray()
            pointarray_msg.name = 'NeedleShapeZ'
            pointarray_msg.pointdata = self.shapedata
            # Push shape to IGTLBridge
            self.publisher_shapeheader.publish(string_msg)
            self.publisher_shape.publish(pointarray_msg)

    # Publishes skin_entry in needle coordinate frame
    # TODO: When possible, replace Point by PointStamped
    def timer_air_gap_needle_callback (self):
        # Publish last needle pose in robot frame
        if (self.air_gap_needle.size != 0):
            msg = Point()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.header.frame_id = 'needle'
            msg = Point(x=self.air_gap_needle[0], y=self.air_gap_needle[1], z=self.air_gap_needle[2])
            self.publisher_air_gap_needle.publish(msg)

    # Publishes needle displacement (x,y,z) in the needle coordinate frame
    def timer_needle_pose_callback (self):
        if (self.needle_pose.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'needle'
            msg.pose.position = Point(x=self.needle_pose[0], y=self.needle_pose[1], z=self.needle_pose[2])
            msg.pose.orientation = Quaternion(w=self.needle_pose[3], x=self.needle_pose[4], y=self.needle_pose[5], z=self.needle_pose[6])
            self.publisher_needle_pose.publish(msg)
            
########################################################################

# Function: pose_transform
# DO: Transform pose to new reference frame
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf*q_orig
    p_new = q_tf*p_orig*q_tf.conj() + p_tf

    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

# Function: pose_inv_transform
# DO: Transform pose to new reference frame with inverse transform 
# Inputs: 
#   x_origin: pose in original reference frame (numpy array [x, y, z, qw, qx, qy, qz])
#   x_tf: transformation from original to new frame (numpy array [x, y, z, qw, qx, qy, qz])
# Output:
#   x_new: pose in new reference frame (numpy array [x, y, z, qw, qx, qy, qz])
def pose_inv_transform(x_orig, x_tf):

    #Define frame transformation
    p_tf = np.quaternion(0, x_tf[0], x_tf[1], x_tf[2])
    q_tf= np.quaternion(x_tf[3], x_tf[4], x_tf[5], x_tf[6])

    #Define original position and orientation
    p_orig = np.quaternion(0, x_orig[0], x_orig[1], x_orig[2])
    q_orig = np.quaternion(x_orig[3], x_orig[4], x_orig[5], x_orig[6])

    #Transform to new frame
    q_new = q_tf.conj()*q_orig
    p_new = q_tf.conj()*(p_orig-p_tf)*q_tf
    x_new = np.array([p_new.x, p_new.y, p_new.z, q_new.w, q_new.x, q_new.y, q_new.z])
    return x_new

########################################################################

def main(args=None):
    rclpy.init(args=args)

    smart_needle_interface = SmartNeedleInterface()

    smart_needle_interface.get_logger().warn('Waiting smart_needle...')
    # Wait for smart_needle
    while rclpy.ok():
        rclpy.spin_once(smart_needle_interface)
        if(smart_needle_interface.tip.size == 0): # Keep loop while stage position not set
            pass
        else:
            break

    rclpy.spin(smart_needle_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    smart_needle_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()