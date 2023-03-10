import getch
import rclpy
import numpy as np
import quaternion

from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion, Point

DEPTH_OFFSET = 195.0                    # Forced initial depth just for testing purposes
INSERTION_LENGTH = 100.0                # Total insertion length = +100mm (negative in stage frame)
INSERTION_STEP = 5.0                    # Insertion depth step = +5mm (negative in stage frame)
ROBOT_STEP = 1.0                        # Robot displacement step = 1mm

class TrajcontrolDemo(Node):

    def __init__(self):
        super().__init__('trajcontrol_demo')

        # Topics from sensorized needle node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.needle_callback,  10)
        self.subscription_sensor # prevent unused variable warning

        #Topic from keypress node
        self.subscription_keyboard = self.create_subscription(Int8, '/keyboard/key', self.keyboard_callback, 10)
        self.subscription_keyboard # prevent unused variable warning
        
        # Published topics
        timer_period = 0.3 # seconds
        self.timer_needle = self.create_timer(timer_period, self.timer_callback)        
        self.publisher_needle = self.create_publisher(PoseStamped,'/stage/state/needle_pose', 10)
        
        # Robot and needle tip (stage frame)
        self.entry_point = np.empty(shape=[0,3])    # Initial needle_pose
        self.stage = np.empty(shape=[0,2])          # Current stage position
        self.depth = 0.0                            # Current insertion depth
        self.registration = np.empty(shape=[0,7])   # Needle to stage coord transformation
        
    # Get current sensor value
    def needle_callback(self, msg_sensor):
        self.get_logger().info('Shape received')
        # From needle shape, get measured tip pose (needle frame)
        shape = msg_sensor.poses
        N = len(shape)
        tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])  # Get tip (last shape point)
        q = np.array([shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])
        sensorZ = np.array([tip[0], tip[1], tip[2], q[0], q[1], q[2], q[3]])
        self.get_logger().info('Tip (needle) = [%f, %f, %f]' %(sensorZ[0], sensorZ[1], sensorZ[2]))
        
        # Convert from needle to stage frame
        if (self.registration.size != 0): 
            Z = pose_transform(sensorZ, self.registration)
            self.get_logger().info('Tip (stage) = [%f, %f, %f]' %( Z[0], Z[1], Z[2]))

    def keyboard_callback(self, msg):
        self.get_logger().info('Keyboard input')
        # Update depth and robot positions
        if (self.entry_point.size == 0):              # Begining of the experiment
            self.depth = 0.0                          # Initial value for needle insertion depth
            self.stage = np.array([0.050, 0.005])     # Initial robot position
            self.entry_point = np.array([self.stage[0], -self.depth, self.stage[1]])
            self.registration = np.concatenate((self.entry_point[0:3], np.array([np.cos(np.deg2rad(45)),np.sin(np.deg2rad(45)),0,0]))) # Registration now comes from entry point
            self.get_logger().info('Entry point = %s' %(self.entry_point))
        else:   
            # Update insertion depth
            self.depth = self.depth + INSERTION_STEP
            # Emulate robot motion with keyboard
            if (msg.data==50):     #Robot down
                self.stage = self.stage + np.array([0.0, -ROBOT_STEP])
            elif (msg.data==52):   #Robot left
                self.stage = self.stage + np.array([-ROBOT_STEP, 0.0])
            elif (msg.data==54):   #Robot right
                self.stage = self.stage + np.array([ROBOT_STEP, 0.0])
            elif (msg.data==56):   #Robot up
                self.stage = self.stage + np.array([0.0, ROBOT_STEP])

    def timer_callback(self):
        # Publish needle_pose topic
        if (self.entry_point.size != 0):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'needle'
            msg.pose.position = Point(x=(self.stage[0]-self.entry_point[0]), y=(self.stage[1]-self.entry_point[2]), z=self.depth+self.entry_point[1]+DEPTH_OFFSET)
            msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            self.publisher_needle.publish(msg)
            self.get_logger().debug('Base (needle) = [%f, %f, %f]' %(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)) 

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

    trajcontrol_demo = TrajcontrolDemo()
    trajcontrol_demo.get_logger().info('**** To START experiment, hit a valid key ****')

    while rclpy.ok():
        rclpy.spin_once(trajcontrol_demo)
        # print('Insertion depth = %f / %f' %(trajcontrol_demo.depth, INSERTION_LENGTH))
        if (trajcontrol_demo.depth) >= INSERTION_LENGTH:
            trajcontrol_demo.get_logger().info('ATTENTION: Insertion depth reached! Please stop insertion') 
            break

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajcontrol_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()