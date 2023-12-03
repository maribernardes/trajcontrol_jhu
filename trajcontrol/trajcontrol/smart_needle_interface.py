import os
import rclpy
import numpy as np
import quaternion
import datetime

from rclpy.node import Node
from numpy import loadtxt
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from ros2_igtl_bridge.msg import PointArray, String
from trajcontrol_interfaces.srv import GetPoint, GetPose
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
# '/needle/state/tip'           (geometry_msgs.msg.PoseStamped)   - robot frame
# '/stage/state/needle_pose'    (geometry_msgs.msg.PoseStamped)   - needle frame
# '/needle/state/skin_entry'    (geometry_msgs.msg.Point)         - needle frame
# 'IGTL_STRING_OUT'             (ros2_igtl_bridge.msg.String)     - zFrame
# 'IGTL_POINT_OUT'              (ros2_igtl_bridge.msg.PointArray) - zFrame
# 
# Service client:    
# '/get_planning_point'             (trajcontrol_interfaces.srv.GetPoint) - robot frame
#  name: "initial_point", "skin_entry" OR "target"
#
# Service server:    
# '/get_needle_tip'             (trajcontrol_interfaces.srv.GetPose) - robot frame
#  OBS: request.name: "tip"
# '/get_needle_base'            (trajcontrol_interfaces.srv.GetPose) - robot frame
#  OBS: request.name: base"
#
#########################################################################

class SmartNeedleInterface(Node):

    def __init__(self):
        super().__init__('smart_needle_interface')

        #Declare node parameters
        self.declare_parameter('use_slicer', False) # Push shape to OpenIGTLink bridge

#### Stored variables ###################################################

        # Node parameters
        self.push_to_bridge = self.get_parameter('use_slicer').get_parameter_value().bool_value

        self.zFrameToRobot = np.empty(shape=[0,7])      # zFrame to robot frame transform (from robot package)
        self.needleToRobot = np.empty(shape=[0,7])      # Needle to robot frame transform (initialized from initial_point)

        self.initial_point = np.empty(shape=[0,3])      # Robot position at begining of experiment (robot frame)
        self.skin_entry = np.empty(shape=[0,3])         # Skin entry at begining of experiment (robot frame)
        self.skin_entry_needle = np.empty(shape=[0,3])  # Skin entry at begining of experiment (needle frame)
        
        self.stage = np.empty(shape=[0,3])              # Robot current position (robot frame)
        self.needle_pose = np.empty(shape=[0,7])        # Base pose (needle frame)
        self.Z = np.empty(shape=[0,7])                  # Tip pose (robot frame)
        self.sensorZ = np.empty(shape=[0,7])            # Tip pose (needle frame)

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

        # Tip (robot frame)
        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)        
        self.publisher_tip = self.create_publisher(PoseStamped,'/needle/state/tip', 10)  #(robot frame)

        # Base (needle frame)
        timer_period_needle_pose = 0.3 # seconds
        self.timer_needle_pose = self.create_timer(timer_period_needle_pose, self.timer_needle_pose_callback)        
        self.publisher_needle_pose = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)  #(needle frame)
        
        # Skin_entry (needle frame)
        timer_period_skin_entry_needle = 0.3 # seconds
        self.timer_skin_entry_needle = self.create_timer(timer_period_skin_entry_needle, self.timer_skin_entry_needle_callback)        
        self.publisher_skin_entry_needle = self.create_publisher(Point, '/needle/state/skin_entry', 10) #(needle frame)

        # Needle shape (zFrame)
        if self.push_to_bridge is True:
            timer_period_shape = 1.0 # seconds
            self.timer_shape = self.create_timer(timer_period_shape, self.timer_shape_callback)        
            self.publisher_shapeheader = self.create_publisher(String, 'IGTL_STRING_OUT', 10)
            self.publisher_shape = self.create_publisher(PointArray, 'IGTL_POINT_OUT', 10)

#### Service client ###################################################

        # Planning service client
        # OBS: Node will not spin until services are available
        self.service_client = self.create_client(GetPoint, '/get_planning_point')
        if not self.service_client.service_is_ready():
            self.get_logger().warn('Planning service server not available, waiting...')
        while not self.service_client.wait_for_service(timeout_sec=5.0):
            pass
        self.get_logger().warn('Planning service available')
            

# #### Service server ###################################################

#         # Service server to return planning points
#         self.service_server_base = None  # Activate only after self.X is available
#         self.service_server_tip = None   # Activate only after self.Z is available

#### Interface initialization ###################################################

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

        # Load zFrameToRobot transform
        try:
            smart_template_share_directory = get_package_share_directory('smart_template')
            self.zFrameToRobot  = np.array(loadtxt(os.path.join(smart_template_share_directory,'files','zframe.csv'), delimiter=','))
        except IOError:
            self.get_logger().info('Could not find zframe.csv file')

        # Initialize planning points
        initial_point = self.get_planning_point('initial_point')
        if initial_point.valid is True:
            self.initial_point = np.array([initial_point.x, initial_point.y, initial_point.z])
        else:
            self.get_logger().info('Invalid initial_point')
        skin_entry = self.get_planning_point('skin_entry')
        if skin_entry.valid is True:
            self.skin_entry = np.array([skin_entry.x, skin_entry.y, skin_entry.z])
        else:
            self.get_logger().info('Invalid skin_entry point')
 
        # Set needleToRobot transform         
        q_tf1= np.quaternion(np.cos(np.deg2rad(-45)), np.sin(np.deg2rad(-45)), 0, 0)
        q_tf2= np.quaternion(np.cos(np.deg2rad(90)), 0, 0, np.sin(np.deg2rad(90)))
        q_tf = q_tf1*q_tf2  
        needle_base= np.array([self.initial_point[0], self.initial_point[1], self.initial_point[2]])  # stage initial_point
        self.needleToRobot = np.concatenate((needle_base, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        # Store skin_entry point in needle frame
        skin_entry_robot = np.array([skin_entry.x, skin_entry.y, skin_entry.z, 1,0,0,0])
        self.skin_entry_needle = pose_inv_transform(skin_entry_robot, self.needleToRobot)[0:3]   # skin_entry in needle frame            

#### Service client functions ###################################################

    # Send Command service to robot 
    def get_planning_point(self, point_name):
        request = GetPoint.Request()
        request.name = point_name
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# #### Service server functions ###################################################

#     # Send requested tip pose in robot frame
#     def get_tip_callback(self, request, response):
#         pose_name = request.name
#         self.get_logger().debug('Received %s request' %(pose_name))
#         if pose_name == 'tip':
#             if (self.Z.size == 0):
#                 response.valid = False
#             else:
#                 response.valid = True
#                 response.x = self.Z[0]
#                 response.y = self.Z[1]
#                 response.z = self.Z[2]
#                 response.qw = self.Z[3]
#                 response.qx = self.Z[4]
#                 response.qy = self.Z[5]
#                 response.qz = self.Z[6]        
#         return response    

#     # Send requested base pose in robot frame
#     def get_base_callback(self, request, response):
#         pose_name = request.name
#         self.get_logger().debug('Received %s request' %(pose_name))
#         if pose_name == 'base':
#             if (self.X.size == 0):
#                 response.valid = False
#             else:
#                 response.valid = True
#                 response.x = self.X[0]
#                 response.y = self.X[1]
#                 response.z = self.X[2]
#                 response.qw = self.X[3]
#                 response.qx = self.X[4]
#                 response.qy = self.X[5]
#                 response.qz = self.X[6]        
#         return response  

#### Listening callbacks ###################################################

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.point
        self.stage = np.array([robot.x, robot.y, robot.z])
        # Store current needle base pose (in robot and needle frames)
        if (self.needleToRobot.size != 0):
            needle_q = self.needleToRobot[3:7]                                      
            needle_base = np.array([self.stage[0], self.stage[1], self.stage[2], needle_q[0], needle_q[1], needle_q[2], needle_q[3]]) # base in robot frame       
            self.needle_pose = pose_inv_transform(needle_base, self.needleToRobot)  # needle base in needle frame

    # Get current sensor measurements
    def shape_callback(self, msg_sensor):
        # From shape, get tip (last point)
        self.shapecount += 1
        shape = msg_sensor.poses      
        N = len(shape) 
        # Tip is last point in shape
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z]) 
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])
        self.sensorZ = np.array([tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]]) # needle tip in needle frame
        # Transform from needle to robot frame
        if (self.needleToRobot.size != 0): 
            self.Z = pose_transform(self.sensorZ, self.needleToRobot)             # needle tip in robot frame
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

    # Publishes needle tip transformed to robot frame
    def timer_tip_callback (self):
        # Publish last needle pose in robot frame
        if (self.Z.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.Z[0], y=self.Z[1], z=self.Z[2])
            msg.pose.orientation = Quaternion(w=self.Z[3], x=self.Z[4], y=self.Z[5], z=self.Z[6])
            self.publisher_tip.publish(msg)

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
    def timer_skin_entry_needle_callback (self):
        # Publish last needle pose in robot frame
        if (self.skin_entry_needle.size != 0):
            msg = Point()
            # msg.header.stamp = self.get_clock().now().to_msg()
            # msg.header.frame_id = 'needle'
            msg = Point(x=self.skin_entry_needle[0], y=self.skin_entry_needle[1], z=self.skin_entry_needle[2])
            self.publisher_skin_entry_needle.publish(msg)

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
    smart_needle_interface.get_logger().info('Received planning points')
    rclpy.spin(smart_needle_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    smart_needle_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()