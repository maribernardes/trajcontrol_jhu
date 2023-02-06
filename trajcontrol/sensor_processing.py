import os
import rclpy
import numpy as np
import quaternion

from rclpy.node import Node
from numpy import loadtxt
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Quaternion, Point
from scipy.ndimage import median_filter

INSERTION_STEP = -5.0        # 5mm insertion step

class SensorProcessing(Node):

    def __init__(self):
        super().__init__('sensor_processing')

        #Declare node parameters
        self.declare_parameter('insertion_length', -100.0) #Insertion length parameter

        #Topics from sensorized needle node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.needle_callback,  10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node (LISA robot)
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10) #CAUTION: no '\'  before topic name
        self.subscription_robot # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

        #Published topics
        timer_period_entry = 1.0  # seconds
        self.timer_entry = self.create_timer(timer_period_entry, self.timer_entry_point_callback)        
        self.publisher_entry_point = self.create_publisher(PointStamped, '/subject/state/skin_entry', 10)
        self.publisher_target = self.create_publisher(PointStamped, '/subject/state/target', 10)

        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/sensor/tip', 10)
        
        ## ADDED: WE ARE PUBLISHING THIS HERE IN PLACE OF THE STAGE (REMOVED LINEAR POTENTIOMETER)
        # Publishing only the absolute insertion depth (in msg y coordinate)
        timer_period_needle = 0.3 # seconds
        self.timer_needle = self.create_timer(timer_period_needle, self.timer_needle_callback)        
        self.publisher_needle = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)

        # When keyboard SPACE is pressed (no timer)
        self.publisher_base = self.create_publisher(PoseStamped,'/sensor/base', 10)

        #Stored values
        self.registration = np.empty(shape=[0,7])
        self.entry_point = np.empty(shape=[0,3])    # Tip position at begining of insertion
        self.sensorZ = np.empty(shape=[0,7])        # All stored sensor tip readings as they are sent (for median filter)
        self.Z = np.empty(shape=[0,7])              # Current tip value (filtered) in robot frame
        self.stage = np.empty(shape=[0,2])          # Current stage pose
        self.depth = 0.0                            # Current insertion depth
        self.X = np.empty(shape=[0,3])              # Needle base position
        self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value
        self.get_logger().info('Final insertion length for this trial: %f' %(self.insertion_length))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        # Get current robot position
        self.stage = np.array([robot.position.x*1000, robot.position.z*1000])

    # Get current sensor measurements
    def needle_callback(self, msg_sensor):
         # Get shape from sensorized needle
        shape = msg_sensor.poses
        self.get_logger().info('Shape received')
        
        # From shape, get measured Z
        N = len(shape)
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  #get tip
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])

        ##########################################
        # TODO: Check use of orientation from shape instead of calculating from last two points
        ##########################################
        # if (N==1):
        #     q = [0, 0, math.cos(math.pi/4), math.cos(math.pi/4)]
        # else:
        #     ptip = np.array([shape[N-2].position.x, shape[N-2].position.y, shape[N-2].position.z]) #prior to tip
        #     forw = tip - ptip
        #     q = upforw2quat([0, 0, 1], forw)
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

        ##########################################
        # TODO: Check new simplified registration
        ##########################################
        # Transform from sensor to robot frame
        if (self.registration.size != 0): 
            self.Z = pose_transform(Z_new, self.registration)
            self.get_logger().debug('Zsensor = (%f, %f, %f)' %(self.sensorZ[0],self.sensorZ[1],self.sensorZ[2]))
            # self.get_logger().info('Zrobot = %s' %(self.Z))
        
    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.listen_keyboard == True) and (msg.data == 32): # If listerning to keyboard and hit SPACE key
            if (self.entry_point.size == 0):              # At needle entry point
                self.depth = 0.0                          # Initial value for needle insertion depth
                self.entry_point = np.array([self.stage[0], self.depth, self.stage[1]])
                self.registration = np.concatenate((self.entry_point[0:3], np.array([np.cos(np.deg2rad(45)),np.sin(np.deg2rad(45)),0,0]))) # Registration now comes from entry point
                self.get_logger().info('Entry point = %s' %(self.entry_point))
            else:
                self.depth = self.depth + INSERTION_STEP
            # Store current base value
            if (self.stage.size != 0):
                self.X = np.array([self.stage[0], self.depth, self.stage[1]])
                # Publish last base filtered pose in robot frame
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'stage'
                msg.pose.position = Point(x=self.X[0], y=self.X[1], z=self.X[2])
                msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
                self.publisher_base.publish(msg)

    def get_entry_point(self):
        # Display message for entry point acquisition
        if (self.entry_point.size == 0) and (self.listen_keyboard == False):
            #Listen to keyboard
            self.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            self.get_logger().info('Place the needle at the Entry Point and hit SPACE bar')
            self.listen_keyboard = True

    # Publishes entry point and target
    def timer_entry_point_callback(self):
        # Publishes only after experiment started (stored entry point is available)
        if (self.entry_point.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "stage"
            msg.point = Point(x=self.entry_point[0], y=self.entry_point[1], z=self.entry_point[2])
            self.publisher_entry_point.publish(msg)
            msg.point = Point(x=self.entry_point[0], y=self.insertion_length, z=self.entry_point[2])
            self.publisher_target.publish(msg)      #Currently target equals entry point x and z (in the future target will be provided by 3DSlicer)

    # Publishes needle displacement (x,y,z) in the needle coordinate frame
    def timer_needle_callback (self):
        if (self.entry_point.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'needle'
            msg.pose.position = Point(x=(self.stage[0]-self.entry_point[0]), y=(self.stage[1]-self.entry_point[2]), z=-self.depth)
            msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            self.publisher_needle.publish(msg)
            self.get_logger().debug('NeedlePose = (%f, %f, %f)' %(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
            
            
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
            # self.get_logger().info('Tip = %s' %(self.Z))

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

def main(args=None):
    rclpy.init(args=args)

    sensor_processing = SensorProcessing()
    
    # Initialize entry point position
    while rclpy.ok():
        rclpy.spin_once(sensor_processing)
        if sensor_processing.entry_point.size == 0: #No entry point yet
            sensor_processing.get_entry_point()
        else:
            sensor_processing.get_logger().info('*****EXPERIMENT STARTED*****\nEntry Point in (%f, %f, %f)' %(sensor_processing.entry_point[0], sensor_processing.entry_point[1], sensor_processing.entry_point[2]))
            sensor_processing.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm, then hit SPACE' % (sensor_processing.depth, INSERTION_STEP))      

            break

    rclpy.spin(sensor_processing)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_processing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
