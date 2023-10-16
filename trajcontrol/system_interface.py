import os
import rclpy
import numpy as np
import quaternion
import datetime


from rclpy.node import Node
from numpy import loadtxt
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from ros2_igtl_bridge.msg import PointArray, String
from scipy.ndimage import median_filter

INSERTION_STEP = -5.0        # 5mm insertion step

class SystemInterface(Node):

    def __init__(self):
        super().__init__('system_interface')

        #Declare node parameters
        self.declare_parameter('insertion_length', -100.0) #Insertion length parameter

        #Topics from 3D Slicer interface (OpenIGTLink Bridge)
        self.subscription_bridge_point = self.create_subscription(PointArray, 'IGTL_POINT_IN', self.bridge_callback, 10)
        self.subscription_bridge_point # prevent unused variable warning

        #Topics from sensorized needle node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.shape_callback,  10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node (robot)
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10) #CAUTION: no '\'  before topic name
        self.subscription_robot # prevent unused variable warning

        #Topics from Arduino
        self.subscription_depth = self.create_subscription(Int16, '/arduino/depth', self.depth_callback, 10)
        self.subscription_depth # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

        #Published topics
        timer_period_initialize = 1.0  # seconds
        self.timer_initialize = self.create_timer(timer_period_initialize, self.timer_initialize_callback)        
        self.publisher_initial_point = self.create_publisher(PointStamped, '/stage/state/initial_point', 10)
        self.publisher_skin_entry = self.create_publisher(PointStamped, '/subject/state/skin_entry', 10)
        self.publisher_target = self.create_publisher(PointStamped, '/subject/state/target', 10)

        timer_period_shape = 1.0 # seconds
        self.timer_shape = self.create_timer(timer_period_shape, self.timer_shape_callback)        
        self.publisher_shapeheader = self.create_publisher(String, 'IGTL_STRING_OUT', 10)
        self.publisher_shape = self.create_publisher(PointArray, 'IGTL_POINT_OUT', 10)

        timer_period_base = 0.3 # seconds
        self.timer_base = self.create_timer(timer_period_base, self.timer_base_callback)
        self.publisher_base = self.create_publisher(PoseStamped,'/sensor/base', 10) #(stage frame)      

        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/sensor/tip', 10)  #(stage frame)

        timer_period_needle_pose = 0.3 # seconds
        self.timer_needle_pose = self.create_timer(timer_period_needle_pose, self.timer_needle_pose_callback)        
        self.publisher_needle_pose = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)   #needle frame

        #Stored values
        self.needleToRobot = np.empty(shape=[0,7])  # Needle to robot frame transform
        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform
        self.shapeheader = None                     # Shape message header to push to 3D Slicer
        self.shapedata = None                       # Shape message data to push to 3D Slicer
        self.shapecount = 0
        
        self.target = np.empty(shape=[0,3])         # Tip position at target
        self.skin_entry = np.empty(shape=[0,3])     # Skin entry point position (from 3D Slicer)
        self.initial_point = np.empty(shape=[0,3])  # Stage position at begining of experiment
        
        self.sensorZ = np.empty(shape=[0,7])        # All stored sensor tip readings as they are sent (for median filter)
        self.Z = np.empty(shape=[0,7])              # Current tip value (filtered) in robot frame

        self.stage = np.empty(shape=[0,2])          # Current stage pose
        self.depth = None                           # Current insertion depth
        self.initial_depth = None                   # Initial insertion depth
        self.X = np.empty(shape=[0,7])              # Needle base position [robot frame]
        self.needle_pose = np.empty(shape=[0,7])    # Needle base pose [needle frame]
        self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value
        self.get_logger().info('Final insertion length for this trial: %f' %(self.insertion_length))

        # Initialize zFrameToRobot transform
        q_tf = np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
        zFrameCenter = np.array([0,0,0])
        self.zFrameToRobot = np.concatenate((zFrameCenter, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

    # Get current skin entry and target points 
    def bridge_callback(self, msg_point):
        name = msg_point.name      
        npoints = len(msg_point.pointdata)
        if (name == 'TARGET') and (npoints == 2): # Name is adjusted in 3DSlicer module
            skin_entry_zFrame = np.array([msg_point.pointdata[0].x, msg_point.pointdata[0].y, msg_point.pointdata[0].z, 1,0,0,0])
            target_zFrame = np.array([msg_point.pointdata[1].x, msg_point.pointdata[1].y, msg_point.pointdata[1].z, 1,0,0,0])
            skin_entry_robot = pose_transform(skin_entry_zFrame, self.zFrameToRobot)
            target_robot = pose_transform(target_zFrame, self.zFrameToRobot)
            self.skin_entry = skin_entry_robot[0:3]
            self.target = target_robot[0:3]
            self.get_logger().info('Skin Entry = (%f, %f, %f)' %(self.skin_entry[0],self.skin_entry[1],self.skin_entry[2]))
            self.get_logger().info('Target = (%f, %f, %f)' %(self.target[0],self.target[1],self.target[2]))

    # Depth value published
    # Initialize and update insertion depth
    def depth_callback(self, msg):
        self.depth = float(msg.data)

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        # Initialize robot
        if (self.stage.size == 0):
            stage_initial = np.array([robot.position.x*1000, robot.position.z*1000])
            self.robot_idle = True                  # Initialize robot status
            self.get_logger().debug('Initial stage position in (%f, %f)' %(stage_initial[0], stage_initial[1])) 
        # Stores current robot position
        self.stage = np.array([robot.position.x*1000, robot.position.z*1000])
        # Store current base pose (in robot and needle frames)
        if (self.needleToRobot.size != 0):
            needle_q = self.needleToRobot[3:7]
            self.X = np.array([self.stage[0], -(self.depth-self.initial_depth), self.stage[1], needle_q[0], needle_q[1], needle_q[2], needle_q[3]]) #base in robot frame       
            self.needle_pose = pose_inv_transform(self.X, self.needleToRobot)   # base in needle frame

    # Get current sensor measurements
    def shape_callback(self, msg_sensor):
        # Store needle tip and shape
        if (self.needleToRobot.size != 0):
            # Get msg from sensorized needle
            self.shapecount += 1
            shape = msg_sensor.poses        
            N = len(shape) # Package size
            frame_id = msg_sensor.header.frame_id
            timestamp = msg_sensor.header.stamp
            # Convert timestamp to a readable format
            now = datetime.datetime.now()
            timestamp_duration = datetime.timedelta(seconds=timestamp.nanosec / 1e9)
            duration_since_epoch = now - timestamp_duration
            # Get the time_t object from the datetime
            time_t_object = datetime.datetime.fromtimestamp(duration_since_epoch.timestamp())
            # Format the timestamp with seconds and milliseconds
            formatted_timestamp = time_t_object.strftime('%Y-%m-%d %H:%M:%S') + '.{:03d}'.format(int(timestamp.nanosec % 1e6 / 1e3))
            # self.get_logger().info('Timestamp: %s' %formatted_timestamp)
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
            # From shape, get measured Z
            tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  #get tip
            q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])
            Z_new = np.array([tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]])
            ##########################################
            # TODO: Check need of filtering sensor data
            ##########################################
            # # Filter and transform sensor data only after registration was loaded from file
            # self.sensorZ = np.row_stack((self.sensorZ, Z_new))
            # # Smooth the measurements with a median filter 
            # n = self.sensorZ.shape[0]
            # size_win = min(n, 500) #array window size
            # if (size_win>0): 
            #     Z_filt = median_filter(self.sensorZ[n-size_win:n,:], size=(40,1))  # use 40 samples median filter (column-wise)
            #     Z_new = Z_filt[size_win-1,:]                                       # get last value
            self.sensorZ = np.copy(Z_new)
            # Transform from sensor to robot frame
            self.Z = pose_transform(Z_new, self.needleToRobot)
                # self.get_logger().info('Zsensor = (%f, %f, %f)' %(self.sensorZ[0],self.sensorZ[1],self.sensorZ[2]))
        
    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.depth is None):
            self.get_logger().info('Depth sensor is not publishing')
        elif (self.stage.size == 0):
            self.get_logger().info('Stage is not publishing')
        elif (self.listen_keyboard == True) and (msg.data == 32): # If listerning to keyboard and hit SPACE key
            if (self.initial_point.size == 0):                      # Initialize experiment initial point and insertion depth
                self.initial_depth = self.depth             
                self.initial_point = np.array([self.stage[0], -(self.depth-self.initial_depth), self.stage[1]])
                q_tf1= np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
                q_tf2= np.quaternion(np.cos(np.deg2rad(45)), 0, 0, np.sin(np.deg2rad(45)))
                q_tf = q_tf1*q_tf2
                # Set needleToRobot frame transform
                self.needleToRobot = np.concatenate((self.initial_point[0:3], np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z]))) # Registration now comes from entry point
                self.get_logger().info('Initial point = %s' %(self.initial_point))
            # Store current base value
            needle_q = self.needleToRobot[3:7]
            self.X = np.array([self.stage[0], -(self.depth-self.initial_depth), self.stage[1], needle_q[0], needle_q[1], needle_q[2], needle_q[3]]) #base in robot frame       
            # Publish last base filtered pose in robot frame
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.X[0], y=self.X[1], z=self.X[2])
            msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            self.publisher_base.publish(msg)
            # self.get_logger().info('X (stage) = %s' %(self.X))

    def get_initial_point(self):
        # Display message for entry point acquisition
        if (self.initial_point.size == 0) and (self.listen_keyboard == False):
            #Listen to keyboard
            self.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            self.get_logger().info('Place the needle at the Entry Point and hit SPACE bar')
            self.listen_keyboard = True

#### Publish ###################################################

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
            self.get_logger().info('Shape Publisher #' + str(self.shapecount))
            self.get_logger().info('Header:' + self.shapeheader)
            

    # Publishes initial point and target
    def timer_initialize_callback(self):
        # Publishes only after experiment started (stored initial point is available)
        if (self.initial_point.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            # Publish robot initial point (robot frame)
            msg.point = Point(x=self.initial_point[0], y=self.initial_point[1], z=self.initial_point[2])
            self.publisher_initial_point.publish(msg)
            # Publish skin entry (robot frame)
            if (self.skin_entry.size != 0):
                msg.point = Point(x=self.skin_entry[0], y=self.skin_entry[1], z=self.skin_entry[2])
                self.publisher_skin_entry.publish(msg) 
            # Publish target (robot frame)
            if (self.target.size != 0):
                msg.point = Point(x=self.target[0], y=self.target[1], z=self.target[2])
                self.publisher_target.publish(msg) 

    # Publishes needle base transformed to robot frame
    def timer_base_callback (self):
        # Publish last needle pose in robot frame
        if (self.X.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.X[0], y=self.X[1], z=self.X[2])
            msg.pose.orientation = Quaternion(w=self.X[3], x=self.X[4], y=self.X[5], z=self.X[6])
            self.publisher_base.publish(msg)

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

    system_interface = SystemInterface()
    system_interface.get_logger().info('Waiting for stage and depth sensor...')

    # Wait for stage and sensor to start publishing
    while rclpy.ok():
        rclpy.spin_once(system_interface)
        if(system_interface.depth is None) or (system_interface.stage.size == 0):
            pass
        else:
            system_interface.get_logger().info('Stage and depth sensor connected. Now place the needle at the Entry Point')
            system_interface.get_logger().info('**** To START experiment, hit SPACE ****')
            system_interface.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            system_interface.listen_keyboard = True
            break

    # Initialize initial point position
    while rclpy.ok():
        rclpy.spin_once(system_interface)
        if system_interface.initial_point.size == 0: #No entry point yet
            pass
        else:
            system_interface.get_logger().info('*****EXPERIMENT STARTED*****')
            break    

    rclpy.spin(system_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    system_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()