import getch
import rclpy
import numpy as np
import quaternion
import datetime

from rclpy.node import Node
from std_msgs.msg import Int8, Int16
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion, Point, PointStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from stage_control_interfaces.action import MoveStage

ROBOT_STEP = 1.0            # Robot displacement step = 1mm
INSERTION_STEP = -20.0      # Needle insertion step = 20mm 
TOTAL_INSERTION = -120.0    # Total needle insertion lenght

class TrajcontrolDemoStep(Node):

    def __init__(self):
        super().__init__('trajcontrol_demo_step')

        #Declare node parameters
        # self.declare_parameter('insertion_length', -100.0) #Insertion length parameter

        #Topics from sensorized needle node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.needle_callback,  10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from robot node (LISA robot)
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10) #CAUTION: no '\'  before topic name
        self.subscription_robot # prevent unused variable warning

        #Topics from Arduino
        self.subscription_depth = self.create_subscription(Int16, '/arduino/depth', self.depth_callback, 10)
        self.subscription_depth # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        self.listen_keyboard = False

        #Action client 
        self.action_client = ActionClient(self, MoveStage, '/move_stage')

        #Published topics
        timer_period_entry = 1.0  # seconds
        self.timer_entry = self.create_timer(timer_period_entry, self.timer_entry_point_callback)        
        self.publisher_entry_point = self.create_publisher(PointStamped, '/subject/state/skin_entry', 10)
        self.publisher_target = self.create_publisher(PointStamped, '/subject/state/target', 10)

        timer_period_needle = 0.3 # seconds
        self.timer_needle = self.create_timer(timer_period_needle, self.timer_needle_callback)        
        self.publisher_needle = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)   #needle frame

        timer_period_tip = 0.3 # seconds
        self.timer_tip = self.create_timer(timer_period_tip, self.timer_tip_callback)
        self.publisher_tip = self.create_publisher(PoseStamped, '/sensor/tip', 10)  #(stage frame)

        timer_period_base = 0.3 # seconds
        self.timer_base = self.create_timer(timer_period_base, self.timer_base_callback)
        self.publisher_base = self.create_publisher(PoseStamped,'/sensor/base', 10) #(stage frame)       

        #Stored values
        self.needleToRobot = np.empty(shape=[0,7])  # Needle to robot frame transform
        self.zFrameToRobot = np.empty(shape=[0,7])  # ZFrame to robot frame transform
        self.shapeheader = None                     # Shape message header to push to 3D Slicer
        self.shapedata = None                       # Shape message data to push to 3D Slicer
        self.stage = np.empty(shape=[2,0])          # Current stage position
        self.robot_idle = False                     # Stage status
        self.initialize_insertion = False           # Flag to initialize the insertion
        self.wait_depth = False                     # Flag to wait for input
        self.wait_sensor = False                    # Flag to wait for input
        self.wait_robot = False                     # Flag to wait for input

        self.entry_point = np.empty(shape=[0,3])    # Tip position at begining of insertion
        self.sensorZ = np.empty(shape=[0,7])        # All stored sensor tip readings as they are sent (for median filter)
        self.Z = np.empty(shape=[0,7])              # Current tip value (filtered) in robot frame
        self.needle_pose = np.empty(shape=[0,7])    # Robot
        self.stage = np.empty(shape=[0,2])          # Current stage pose
        self.depth = None                           # Current insertion depth
        self.initial_depth = None                   # Initial insertion depth
        self.X = np.empty(shape=[0,7])              # Needle base position

        # Initialize zFrameToRobot transform
        q_tf = np.quaternion(np.cos(np.deg2rad(45)), np.sin(np.deg2rad(45)), 0, 0)
        zFrameCenter = np.array([0,0,0])
        self.zFrameToRobot = np.concatenate((zFrameCenter, np.array([q_tf.w, q_tf.x, q_tf.y, q_tf.z])))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

    # Depth value published
    # Update insertion depth and initialize insertion
    def depth_callback(self, msg):
        self.depth = float(msg.data)
        if (self.initialize_insertion is True):
            self.initialize_insertion = False
            # Set initial_depth
            self.initial_depth = self.depth
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
        if (self.wait_depth is True):
            self.wait_depth = False
            self.wait_robot = True
            self.get_logger().info('Depth count: %.1fmm. Please insert additional %.1fmm, then hit key' % (self.depth-self.initial_depth, -INSERTION_STEP))    
                

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
        # Print experimental output
        if self.wait_robot is True:
            self.wait_robot = False
            self.wait_sensor = True
            self.get_logger().info('Base (stage) = %s' %(self.X))
            self.get_logger().info('Base (needle) = %s' %(self.needle_pose))

    # Get current needle shape measurements
    def needle_callback(self, msg_sensor):
         # Get msg from sensorized needle
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
        # From shape, get measured Z
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  #get tip
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])
        Z_new = np.array([tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]])
        self.sensorZ = np.copy(Z_new)
        # Transform from sensor to robot frame
        if (self.needleToRobot.size != 0): 
            self.Z = pose_transform(Z_new, self.needleToRobot)
            self.get_logger().debug('Zsensor = (%f, %f, %f)' %(self.sensorZ[0],self.sensorZ[1],self.sensorZ[2]))
        # Print values
        if self.wait_sensor is True:
            self.wait_sensor = False
            self.get_logger().info('Tip (stage) = %s' %(self.Z))
            self.get_logger().info('Tip (needle) = %s' %(self.sensorZ))

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

    # Publishes entry point and target
    def timer_entry_point_callback(self):
        # Publishes only after experiment started (stored entry point is available)
        if (self.entry_point.size != 0):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.point = Point(x=self.entry_point[0], y=self.entry_point[1], z=self.entry_point[2])
            self.publisher_entry_point.publish(msg)
            msg.point = Point(x=self.entry_point[0], y=TOTAL_INSERTION, z=self.entry_point[2])
            self.publisher_target.publish(msg)      #Currently target equals entry point x and z (in the future target will be provided by 3DSlicer)
    
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
            self.wait_depth = True
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
            trajcontrol_demo_step.get_logger().info('Stage and depth sensor connected. Now place the needle at the Entry Point')
            trajcontrol_demo_step.get_logger().info('**** To START experiment, hit SPACE ****')
            trajcontrol_demo_step.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            trajcontrol_demo_step.listen_keyboard = True
            break

    # Initialize insertion
    while rclpy.ok():
        rclpy.spin_once(trajcontrol_demo_step)
        if trajcontrol_demo_step.needleToRobot.size == 0: # Not initialized yet
            pass
        else:
            trajcontrol_demo_step.get_logger().info('*****EXPERIMENT STARTED*****')
            break

    rclpy.spin(trajcontrol_demo_step)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajcontrol_demo_step.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()