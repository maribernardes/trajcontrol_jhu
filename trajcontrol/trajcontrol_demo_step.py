import getch
import rclpy
import numpy as np
import quaternion

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
        self.stage_initial = np.empty(shape=[2,0])  # Stage home position
        self.stage = np.empty(shape=[2,0])          # Current stage pose
        self.robot_idle = False                     # Stage status

        self.registration = np.empty(shape=[0,7])
        self.entry_point = np.empty(shape=[0,3])    # Tip position at begining of insertion
        self.sensorZ = np.empty(shape=[0,7])        # All stored sensor tip readings as they are sent (for median filter)
        self.Z = np.empty(shape=[0,7])              # Current tip value (filtered) in robot frame
        self.needle_pose = np.empty(shape=[0,3]) 
        self.stage = np.empty(shape=[0,2])          # Current stage pose
        self.depth = None                           # Current insertion depth
        self.initial_depth = None                   # Initial insertion depth
        self.X = np.empty(shape=[0,3])              # Needle base position
        # self.insertion_length = self.get_parameter('insertion_length').get_parameter_value().double_value
        # self.get_logger().info('Final insertion length for this trial: %f' %(self.insertion_length))

        # Print numpy floats with only 3 decimal places
        np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

    # Get current robot pose
    def robot_callback(self, msg_robot):
        robot = msg_robot.pose
        # Stores robot initial position (only once)
        if (self.stage_initial.size == 0):
            self.stage_initial = np.array([robot.position.x*1000, robot.position.z*1000])
            self.robot_idle = True                  # Initialize robot status
            self.get_logger().debug('Initial stage position in (%f, %f)' %(self.stage_initial[0], self.stage_initial[1])) 
        # Stores current robot position
        self.stage = np.array([robot.position.x*1000, robot.position.z*1000])

    # Depth value published
    # Initialize and update insertion depth
    def depth_callback(self, msg):
        if (self.depth is None):    # Update initial depth
            self.initial_depth = float(msg.data)
        else:                       # Update depth value
            self.depth = float(msg.data) - self.initial_depth

    # Get current sensor measurements
    def needle_callback(self, msg_sensor):
        # Get shape from sensorized needle
        shape = msg_sensor.poses
        # From shape, get measured Z (needle frame)
        N = len(shape)
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])                                   # tip position
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])  # tip orientation
        Z_new = np.array([tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]])  
        self.sensorZ = np.copy(Z_new)
        # Transform from sensor to robot frame
        if (self.registration.size != 0): 
            self.Z = pose_transform(Z_new, self.registration)
            self.get_logger().debug('Z (stage) = %s' %(self.Z))

    # A keyboard hotkey was pressed 
    def keyboard_callback(self, msg):
        if (self.initial_depth is None):
            self.get_logger().info('Wait. Depth sensor is not publishing yet')
        elif (self.stage.size == 0):
            self.get_logger().info('Wait. Stage is not publishing yet')
        elif (self.listen_keyboard == True) : # If listerning to keyboard
            if (self.entry_point.size == 0) and (msg.data == 32): # SPACE: initialize needle entry point and insertion depth
                self.depth = 0.0
                self.entry_point = np.array([self.stage[0], self.depth, self.stage[1]])
                self.registration = np.concatenate((self.entry_point[0:3], np.array([np.cos(np.deg2rad(45)),np.sin(np.deg2rad(45)),0,0]))) # Registration now comes from entry point
                self.get_logger().debug('Entry point = %s' %(self.entry_point))
            elif (self.robot_idle == True): # Only takes new control input after converged to previous
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
                self.get_logger().info('Base (stage) = %s' %(self.X))
                self.get_logger().info('Tip (stage) = %s' %(self.Z))
                self.get_logger().info('Base (needle) = %s' %(self.needle_pose))
                self.get_logger().info('Tip (needle) = %s' %(self.sensorZ))


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
        if (self.stage.size != 0) and (self.depth is not None):
            self.X = np.array([self.stage[0], -self.depth, self.stage[1]]) 
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'stage'
            msg.pose.position = Point(x=self.X[0], y=self.X[1], z=self.X[2])
            msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
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
            self.get_logger().debug('Z (stage) = %s' %(self.Z))

    # Publishes needle displacement (x,y,z) in the needle coordinate frame
    def timer_needle_callback (self):
        if (self.entry_point.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'needle'
            self.needle_pose = np.array([self.stage[1]-self.entry_point[2], -(self.stage[0]-self.entry_point[0]), self.depth])
            msg.pose.position = Point(x=(self.stage[1]-self.entry_point[2]), y=-(self.stage[0]-self.entry_point[0]), z=self.depth)
            # msg.pose.position = Point(x=(self.stage[0]-self.entry_point[0]), y=(self.stage[1]-self.entry_point[2]), z=self.depth)
            #TODO: Transform for rotated the needle
            msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            self.publisher_needle.publish(msg)
            self.get_logger().debug('Needle Pose (needle) = (%f, %f, %f)' %(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

    # Send MoveStage action to Stage
    def send_cmd(self, x, z):
        # Send command to stage (convert mm to m)
        self.robot_idle = False     # Set robot status to NOT IDLE
        goal_msg = MoveStage.Goal()
        goal_msg.x = float(x*0.001)
        goal_msg.z = float(z*0.001)
        goal_msg.eps = 0.0001
        self.get_logger().info('Send goal request... Control u: x=%f, z=%f' % (x, z))

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
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.x*1000))
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

def main(args=None):
    rclpy.init(args=args)

    trajcontrol_demo_step = TrajcontrolDemoStep()
    trajcontrol_demo_step.get_logger().info('Waiting for stage and depth sensor...')

    # Wait for stage and sensor publishing
    while rclpy.ok():
        rclpy.spin_once(trajcontrol_demo_step)
        if(trajcontrol_demo_step.initial_depth is None) or (trajcontrol_demo_step.stage.size == 0):
            # trajcontrol_demo_step.get_logger().info(trajcontrol_demo_step.depth)
            # trajcontrol_demo_step.get_logger().info(trajcontrol_demo_step.stage)
            pass
        else:
            trajcontrol_demo_step.get_logger().info('Stage and depth sensor connected. Now place the needle at the Entry Point')
            trajcontrol_demo_step.get_logger().info('**** To START experiment, hit SPACE ****')
            trajcontrol_demo_step.get_logger().info('REMEMBER: Use another terminal to run keypress node')
            trajcontrol_demo_step.listen_keyboard = True
            break

    # Initialize entry point position
    while rclpy.ok():
        rclpy.spin_once(trajcontrol_demo_step)
        if trajcontrol_demo_step.entry_point.size == 0: #No entry point yet
            pass
        else:
            trajcontrol_demo_step.get_logger().info('*****EXPERIMENT STARTED*****\nEntry Point in (%f, %f, %f)' %(trajcontrol_demo_step.entry_point[0], trajcontrol_demo_step.entry_point[1], trajcontrol_demo_step.entry_point[2]))
            trajcontrol_demo_step.get_logger().info('Depth count: %.1fmm. Please insert %.1fmm, then hit SPACE' % (trajcontrol_demo_step.depth, INSERTION_STEP))      
            break

    rclpy.spin(trajcontrol_demo_step)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajcontrol_demo_step.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()