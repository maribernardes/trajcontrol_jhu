import rclpy
import os
import csv
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from numpy import asarray


class SaveFile(Node):

    def __init__(self):
        super().__init__('save_file')
        
        #Declare node parameters
        self.declare_parameter('filename', 'my_data') #Name of file where data values are saved
        self.filename = os.path.join(os.getcwd(),'src','trajcontrol','data',self.get_parameter('filename').get_parameter_value().string_value + '.csv') #String with full path to file

        #Topics from virtual needle
        #TODO Save shape array

        #Topics from sensor processing node
        self.subscription_sensortip = self.create_subscription(PoseStamped, '/sensor/tip', self.sensortip_callback, 10)
        self.subscription_sensortip # prevent unused variable warning

        self.subscription_sensorbase = self.create_subscription(PoseStamped, '/sensor/base', self.sensorbase_callback, 10)
        self.subscription_sensorbase # prevent unused variable warning

        #Topics from UI (currently sensor processing is doing the job)
        self.subscription_entry = self.create_subscription(PointStamped, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_entry  # prevent unused variable warning

        self.subscription_target = self.create_subscription(PointStamped, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Topics from estimator_node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.estimator_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Topics from controller_node
        self.subscription_controller = self.create_subscription(PointStamped, '/stage/control/cmd', self.control_callback, 10)
        self.subscription_controller  # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, 'stage/state/pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Published topics
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.write_file_callback)

        #Array of data
        header = ['Timestamp sec', 'Timestamp nanosec', \
            'Target x', 'Target y', 'Target z', ' Target sec', ' Target nanosec', \
            'Entry_point x', 'Entry_point y', 'Entry_point z', ' Entry_point sec', ' Entry_point nanosec', \
            'Tip x', 'Tip y', 'Tip z', 'Tip qw', 'Tip qx', 'Tip qy', 'Tip qz', 'Tip sec', 'Tip nanosec', \
            'Base x', 'Base y', 'Base z', 'Base qw', 'Base qx', 'Base qy', 'Base qz', 'Base sec', 'Base nanosec',
            'J00', 'J01', 'J02', \
            'J10', 'J11', 'J12', \
            'J20', 'J21', 'J22', \
            'J30', 'J31', 'J32', \
            'J40', 'J41', 'J42', 'J sec', 'J nanosec', \
            'Control x', 'Control y', 'Control z', 'Control sec', 'Control nanosec', \
            'Robot x', 'Robot z', 'Robot sec', 'Robot nanosec'
            
        ]
        
        with open(self.filename, 'w', newline='', encoding='UTF8') as f: # open the file in the write mode
            writer = csv.writer(f)  # create the csv writer
            writer.writerow(header) # write a row to the csv file

        #Last data received
        self.target = [0,0,0, 0,0]                  #target point + sec nanosec (sensor_processing - UI)
        self.entry_point = [0,0,0, 0,0]             #skin entry point + sec nanosec (sensor_processing - UI)
        self.tip = [0,0,0,0,0,0,0, 0,0]        #/sensor/tip (x, y, z, qw, qx, qy, qz) + sec nanosec (sensor_processing)
        self.base = [0,0,0,0,0,0,0, 0,0]       #/sensor/base (x, y(depth), z, 1, 0, 0, 0) + sec nanosec (sensor_processing)
        self.J = [0,0,0, \
                  0,0,0, \
                  0,0,0, \
                  0,0,0, \
                  0,0,0]            # /needle/state/jacobian Matrix (estimator)
        self.Jtime = [0,0]          # Jacobian sec nanosec
        self.cmd = [0,0,0, 0,0]     # /stage/control/cmd (x,y,z) + sec nanosec (sensor_processing)
        self.stage = [0,0, 0,0]     # stage/state/pose (x,z) + sec nanosec  (stage_control)
        self.get_logger().info('Log data will be saved at %s' %(self.filename))   


    #Get current target
    def target_callback(self, msg):
        self.target = [msg.point.x, msg.point.y, msg.point.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]

    #Get current entry_point
    def entry_point_callback(self, msg):
        self.entry_point = [msg.point.x, msg.point.y, msg.point.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]

    #Get current Z (filtered and in robot frame)
    def sensortip_callback(self, msg):
        tip = msg.pose
        self.tip = [tip.position.x, tip.position.y, tip.position.z, \
            tip.orientation.w, tip.orientation.x, tip.orientation.y, tip.orientation.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        # self.get_logger().info('Received tip = %s in %s frame' % (self.tip, msg.header.frame_id))

    #Get current X
    def sensorbase_callback(self, msg):
        base = msg.pose
        self.base = [base.position.x, base.position.y, base.position.z, \
            base.orientation.w, base.orientation.x, base.orientation.y, base.orientation.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        # self.get_logger().info('Received base = %s in %s frame' % (self.base, msg.header.frame_id))
        
    #Get current J
    def estimator_callback(self,msg):
        self.J = np.array(CvBridge().imgmsg_to_cv2(msg)).flatten()
        self.Jtime = [int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        # self.get_logger().info('Received J = %s' % (self.J))

    #Get current control output
    def control_callback(self,msg):
        self.cmd = [msg.point.x, msg.point.y, msg.point.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]        
        # self.get_logger().info('Received cmd' % (self.cmd))

    #Get current robot pose
    def robot_callback(self,msg):
        self.stage = [msg.pose.position.x, msg.pose.position.z, int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]     
        # self.get_logger().info('Received stage' % (self.stage))

    #Save data do file
    def write_file_callback(self):
        now = self.get_clock().now().to_msg()
       
        data = [now.sec, now.nanosec, \
            self.target[0], self.target[1], self.target[2], self.target[3], self.target[4],\
            self.entry_point[0], self.entry_point[1], self.entry_point[2], self.entry_point[3], self.entry_point[4],\
            self.tip[0], self.tip[1], self.tip[2], self.tip[3], self.tip[4], self.tip[5], self.tip[6], self.tip[7], self.tip[8], \
            self.base[0], self.base[1], self.base[2], self.base[3], self.base[4], self.base[5], self.base[6], self.base[7], self.base[8],\
            self.J[0], self.J[1], self.J[2], \
            self.J[3], self.J[4], self.J[5], \
            self.J[6], self.J[7], self.J[8], \
            self.J[9], self.J[10], self.J[11], \
            self.J[12], self.J[13], self.J[14], self.Jtime[0] , self.Jtime[1], \
            self.cmd[0], self.cmd[1], self.cmd[2], self.cmd[3], self.cmd[4], \
            self.stage[0], self.stage[1], self.stage[2], self.stage[3], \
            ]
        
        with open(self.filename, 'a', newline='', encoding='UTF8') as f: # open the file in append mode
            writer = csv.writer(f) # create the csv writer
            writer.writerow(data)  # append a new row to the existing csv file     

def main(args=None):
    rclpy.init(args=args)

    save_file = SaveFile()

    rclpy.spin(save_file)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    save_file.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
