import getch
import rclpy
import numpy as np
import quaternion
import datetime
import message_filters

from rclpy.node import Node
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion, Point, PointStamped
from ros2_igtl_bridge.msg import PointArray, String
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from stage_control_interfaces.action import MoveStage

ROBOT_STEP = 1.0            # Robot displacement step = 1mm
INSERTION_STEP = -20.0      # Needle insertion step = 20mm 
TOTAL_INSERTION = -120.0    # Total needle insertion lenght
NEEDLE_LENGTH = 200.0       # Needle size
DEPTH_ERROR = 5.0           # Depth sensor error       

class TrajcontrolDemoStep(Node):

    def __init__(self):
        super().__init__('trajcontrol_demo_step')

        #Declare node parameters
        # self.declare_parameter('insertion_length', -100.0) #Insertion length parameter

#### Subscribed topics ###################################################

        #Topics from 3D Slicer interface (ros2_iglt_bridge node)
        self.subscription_bridge_point = self.create_subscription(PointArray, 'IGTL_POINT_IN', self.bridge_callback, 10)
        self.subscription_bridge_point # prevent unused variable warning
        
        #Topics from sensorized needle node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.needle_callback,  10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node (LISA robot)
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10) #CAUTION: no '\'  before topic name
        self.subscription_robot # prevent unused variable warning

        #Topics from depth_measurement node (Arduino)
        self.subscription_depth = self.create_subscription(Int16, '/arduino/depth', self.depth_callback, 10)
        self.subscription_depth # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

        #Action client 
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

#### Published topics ###################################################

        # Skin entry and target (robot frame)
        timer_period_planning = 1.0  # seconds
        self.timer_planning = self.create_timer(timer_period_planning, self.timer_planning_callback)        
        self.publisher_skin_entry = self.create_publisher(PointStamped, '/subject/state/skin_entry', 10)
        self.publisher_target = self.create_publisher(PointStamped, '/subject/state/target', 10)

        # Needle shape (zFrame)
        timer_period_shape = 1.0 # seconds
        self.timer_shape = self.create_timer(timer_period_shape, self.timer_shape_callback)        
        self.publisher_shapeheader = self.create_publisher(String, 'IGTL_STRING_OUT', 10)
        self.publisher_shape = self.create_publisher(PointArray, 'IGTL_POINT_OUT', 10)

        # Base (robot frame)
        timer_period_base = 0.3 # seconds
        self.timer_base = self.create_timer(timer_period_base, self.timer_base_callback)
        self.publisher_base = self.create_publisher(PoseStamped,'/sensor/base', 10) #(stage frame)     
        
        # Tip (robot frame)
        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/sensor/tip', 10)  #(stage frame)

        # Base (needle frame)
        timer_period_needle = 0.3 # seconds
        self.timer_needle = self.create_timer(timer_period_needle, self.timer_needle_callback)        
        self.publisher_needle = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)   #(needle frame)

        # Print current values (experiment visual feedback): Syncronized topics (needle_pose and current_shape)
        self.subscription_robot_sync = message_filters.Subscriber(self, PoseStamped, '/stage/state/needle_pose')
        self.subscription_sensor_sync = message_filters.Subscriber(self, PoseArray, '/needle/state/current_shape')
        self.timeSync = message_filters.ApproximateTimeSynchronizer([self.subscription_robot_sync, self.subscription_sensor_sync], 10, 0.1)
        self.timeSync.registerCallback(self.print_sync_callback)  

#### Stored variables ###################################################
        # Frame transformations
        self.needleToRobot = np.empty(shape=[0,7])  # Needle to robot frame transform
        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform

        # 3D Slicer variables
        self.shapeheader = None                     # Shape message header to push to 3D Slicer
        self.shapedata = None                       # Shape message data to push to 3D Slicer
        self.skin_entry = np.empty(shape=[0,3])     # User-defined tip position at skin entry
        self.target = np.empty(shape=[0,3])         # User-defined tip position at desired target

        # Control inputs/outputs
        self.stage = np.empty(shape=[0,2])          # Stage positions: horizontal / vertical
        self.X = np.empty(shape=[0,7])              # Base pose (robot frame)
        self.needle_pose = np.empty(shape=[0,7])    # Base pose (needle frame)
        self.Z = np.empty(shape=[0,7])              # Tip pose (robot frame)
        self.sensorZ = np.empty(shape=[0,7])        # Tip pose (needle frame)

        # Depth sensor
        self.depth = None                           # Current insertion depth
        self.initial_depth = None                   # Initial insertion depth
        self.entry_point = np.empty(shape=[0,3])    # Needle guide position at begining of insertion

        # Flags
        self.robot_idle = False                     # Stage status
        self.initialize_insertion = False           # Flag to initialize the insertion
        self.print_values = False                   # Flag to print experiment values after each insertion step

        # Fix for getting tip values after insertion step update
        self.step_depth = None                      # Expected needle depth value for current insertion step
        self.wait_needle_depth = False              # Flag to wait update in needle depth (because it is too slow)              

#### Interface initialization ###################################################

        # Initialize zFrameToRobot transform
        q_tf = np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
        zFrameCenter = np.array([0,0,0])
        self.zFrameToRobot = np.concatenate((zFrameCenter, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

#### Listening callbacks ###################################################

    # Update insertion depth and initialize insertion
    def depth_callback(self, msg):
        self.depth = float(msg.data)
        if (self.initialize_insertion is True):
            self.initialize_insertion = False
            # Set initial_depth
            self.initial_depth = self.depth
            self.step_depth = 0.0
            # Set entry_point
            self.entry_point = np.array([self.stage[0], self.depth-self.initial_depth, self.stage[1]])
            q_tf1= np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
            q_tf2= np.quaternion(np.cos(np.deg2rad(45)), 0, 0, np.sin(np.deg2rad(45)))
            q_tf = q_tf1*q_tf2
            # Set needleToRobot frame transform
            self.needleToRobot = np.concatenate((self.entry_point[0:3], np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z]))) # Registration now comes from entry point
            self.get_logger().info('*** Initialization step ***')
            self.get_logger().info('Entry point = %s' %(self.entry_point))
            self.get_logger().info('Initial depth = %s' %self.initial_depth)
            self.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm, then hit key' % (self.depth-self.initial_depth, -INSERTION_STEP))                    

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
        if (self.needleToRobot.size != 0): # Only after initialization
            needle_q = self.needleToRobot[3:7]
            self.X = np.array([self.stage[0], -(self.depth-self.initial_depth), self.stage[1], needle_q[0], needle_q[1], needle_q[2], needle_q[3]]) #base in robot frame       
            self.needle_pose = pose_inv_transform(self.X, self.needleToRobot)   # base in needle frame

    # Get current needle shape measurements
    def needle_callback(self, msg_sensor):
        # Get msg from sensorized needle
        shape = msg_sensor.poses      
        N = len(shape) # Package size  
        # From shape, get measured Z
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  #get tip
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])
        Z_new = np.array([tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]])
        self.sensorZ = np.copy(Z_new)
        # Check if data has different depth value (new insertion step)
        if self.wait_needle_depth is True:
            if (abs(self.sensorZ[2] - (self.step_depth+NEEDLE_LENGTH)) < DEPTH_ERROR): #Wait until shape tip at expected depth (with a margin of DEPTH_ERROR)
                self.wait_needle_depth = False
        if (self.needleToRobot.size != 0):  # Only after initialization
            # Transform from sensor to robot frame
            self.Z = pose_transform(Z_new, self.needleToRobot)
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
            self.shapeheader = formatted_timestamp + ';' + str(N) + ';' + frame_id
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

    # Get current entry and target points 
    def bridge_callback(self, msg_point):
        if (self.needleToRobot.size != 0):  # Only after initialization
            name = msg_point.name      
            npoints = len(msg_point.pointdata)
            self.get_logger().info('Message type - %s with %d points' %(name, npoints))
            if (name == 'PlanningZ') and (npoints == 2): # Name is adjusted in 3DSlicer module
                skin_zFrame = np.array([msg_point.pointdata[0].x, msg_point.pointdata[0].y, msg_point.pointdata[0].z])
                target_zFrame = np.array([msg_point.pointdata[1].x, msg_point.pointdata[1].y, msg_point.pointdata[1].z])
                self.skin_entry = pose_transform(skin_zFrame, self.zFrameToRobot)
                self.target = pose_transform(target_zFrame, self.zFrameToRobot)
                self.get_logger().debug('Skin entry = %s' %(self.skin_entry))
                self.get_logger().debug('Target = %s' %(self.target))

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.depth is None):
            self.get_logger().info('Wait. Depth sensor is not publishing yet')
        elif (self.stage.size == 0):
            self.get_logger().info('Wait. Stage is not publishing yet')
        elif (self.listen_keyboard == True) : # If listerning to keyboard
            if (self.needleToRobot.size == 0) and (msg.data == 32):  # SPACE: initialize needle entry point and insertion depth
                self.initialize_insertion = True
            elif (self.needleToRobot.size != 0) and (self.robot_idle == True): # Only takes new control input after converged to previous
                if (self.sensorZ.size != 0):
                    self.wait_needle_depth = True 
                    self.step_depth = self.step_depth - INSERTION_STEP
                x = self.stage[0]
                z = self.stage[1]
                if (msg.data == 50): # move down
                    z = z - ROBOT_STEP
                    key = 'DOWN'
                elif (msg.data == 52): # move left
                    x = x - ROBOT_STEP
                    key = 'LEFT'
                elif (msg.data == 54): # move right
                    x = x + ROBOT_STEP
                    key = 'RIGHT'
                elif (msg.data == 56): # move up
                    z = z + ROBOT_STEP
                    key = 'UP'
                else:
                    key = 'SPACE'
                # Send command to stage
                self.send_cmd(x, z)  
                # Print for experiment output
                self.get_logger().info('*** Insertion step ***')
                self.get_logger().info('Key = %s' %(key))

    def print_sync_callback(self, msg_robot, msg_sensor):
        # Print experimental output
        if (self.print_values is True) and (self.wait_needle_depth is False):
            self.print_values = False
            self.get_logger().info('Base (stage) = %s' %(self.X))
            self.get_logger().info('Base (needle) = %s' %(self.needle_pose))
            self.get_logger().info('Tip (stage) = %s' %(self.Z))
            self.get_logger().info('Tip (needle) = %s' %(self.sensorZ))
            self.get_logger().info('Depth count: %.1fmm. Please insert additional %.1fmm, then hit key' % (self.depth-self.initial_depth, -INSERTION_STEP))    

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

    # Publishes skin entry and target points
    def timer_planning_callback(self):
        # Publishes only after experiment started (stored entry point is available)
        if (self.skin_entry.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.point = Point(x=self.skin_entry[0], y=self.skin_entry[1], z=self.skin_entry[2])
            self.publisher_skin_entry.publish(msg)
            self.get_logger().debug('Skin entry (stage) = %s' %(self.skin_entry))
        if (self.target.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.point = Point(x=self.target[0], y=self.target[1], z=self.target[2])
            self.target.publish(msg)
            self.get_logger().debug('Target (stage) = %s' %(self.target))

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
            self.get_logger().debug('Base (stage) = %s' %(self.X))

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
            self.get_logger().debug('Tip (stage) = %s' %(self.Z))

    # Publishes needle displacement (x,y,z) in the needle coordinate frame
    def timer_needle_callback (self):
        if (self.needle_pose.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'needle'
            msg.pose.position = Point(x=self.needle_pose[0], y=self.needle_pose[1], z=self.needle_pose[2])
            msg.pose.orientation = Quaternion(w=self.needle_pose[3], x=self.needle_pose[4], y=self.needle_pose[5], z=self.needle_pose[6])
            self.publisher_needle.publish(msg)
            self.get_logger().debug('Base (needle) = %s' %(self.needle_pose))

    # Send MoveStage action to Stage
    def send_cmd(self, x, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x*0.001)
        goal_msg.z = float(z*0.001)
        goal_msg.eps = 0.0001
        self.get_logger().debug('Send goal request... Control u: x=%f, z=%f' % (x, z))

        # Wait for action server
        self.action_client.wait_for_server()        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    # Check if MoveStage action was accepted 
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Get MoveStage action finish message (Result)
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.robot_idle = True       # Set robot status to IDLE
            self.print_values = True
            self.get_logger().debug('Goal succeeded! Result: {0}'.format(result.x*1000))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

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

    trajcontrol_demo_step = TrajcontrolDemoStep()
    trajcontrol_demo_step.get_logger().info('Waiting for stage and depth sensor...')

    # Wait for stage and sensor to start publishing
    while rclpy.ok():
        rclpy.spin_once(trajcontrol_demo_step)
        if(trajcontrol_demo_step.depth is None) or (trajcontrol_demo_step.stage.size == 0):
            pass
        else:
            trajcontrol_demo_step.get_logger().info('Stage and depth sensor connected.')
            trajcontrol_demo_step.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            trajcontrol_demo_step.get_logger().info('**** To initialize experiment, place needle at initial position and hit SPACE ****')
            trajcontrol_demo_step.listen_keyboard = True
            break

    # Initialize insertion
    while rclpy.ok():
        rclpy.spin_once(trajcontrol_demo_step)
        if trajcontrol_demo_step.needleToRobot.size == 0: # Not initialized yet
            pass
        else:
            trajcontrol_demo_step.get_logger().info('*****START NEEDLE INSERTION*****')
            break

    rclpy.spin(trajcontrol_demo_step)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajcontrol_demo_step.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()