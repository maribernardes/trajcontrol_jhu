import rclpy
import os
import csv
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from numpy import asarray

class SaveFile(Node):

    def __init__(self):
        super().__init__('save_file')

        
        #Declare node parameters
        self.declare_parameter('filename', 'my_data') #Name of file where data values are saved
        self.filename = os.path.join(os.getcwd(),'src','trajcontrol','data',self.get_parameter('filename').get_parameter_value().string_value + '.csv') #String with full path to file

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.robot_callback, 10)
        self.subscription_robot # prevent unused variable warning

        #Topics from sensorized needle
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/current_shape', self.sensor_callback, 10)
        self.subscription_sensor # prevent unused variable warning

        #Topics from estimator_node
        self.subscription_estimator = self.create_subscription(Image, '/needle/state/jacobian', self.estimator_callback, 10)
        self.subscription_estimator  # prevent unused variable warning

        #Topics from controller_node
        self.subscription_controller = self.create_subscription(PointStamped, '/stage/control/cmd', self.control_callback, 10)
        self.subscription_controller  # prevent unused variable warning

        #Topics from UI
        self.subscription_entry_point = self.create_subscription(Point, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_entry_point  # prevent unused variable warning
        self.subscription_target = self.create_subscription(Point, '/subject/state/target', self.target_callback, 10)
        self.subscription_target  # prevent unused variable warning

        #Timer for saving
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.write_file_callback)

        #Array of data
        header = ['Timestamp sec', 'Timestamp nanosec', \
            'Entry_point x', 'Entry_point y', 'Entry_point z', \
            'Target x', 'Target y', 'Target z', \
            'Tip x', 'Tip y', 'Tip z', 'Tip qw', 'Tip qx', 'Tip qy', 'Tip qz',
            'Base x', 'Base y', 'Base z', 'Base qw', 'Base qx', 'Base qy', 'Base qz',
            'J00', 'J01', 'J02', 'J03', 'J04', 'J05', 'J06', \
            'J10', 'J11', 'J12', 'J13', 'J14', 'J15', 'J16', \
            'J20', 'J21', 'J22', 'J23', 'J24', 'J25', 'J26', \
            'J30', 'J31', 'J32', 'J33', 'J34', 'J35', 'J36', \
            'J40', 'J41', 'J42', 'J43', 'J44', 'J45', 'J46', \
            'J50', 'J51', 'J52', 'J53', 'J54', 'J55', 'J56', \
            'J60', 'J61', 'J62', 'J63', 'J64', 'J65', 'J66', \
            'Control_x', 'Control_z'    
        ]
        
        with open(self.filename, 'w', newline='', encoding='UTF8') as f: # open the file in the write mode
            writer = csv.writer(f)  # create the csv writer
            writer.writerow(header) # write a row to the csv file

        #Last data received
        self.entry_point = [0,0,0]      #skin entry point
        self.target = [0,0,0]           #target point
        self.Z = [0,0,0,0,0,0,0]        #needle tip
        self.X = [0,0,0,0,0,0,0]        #robot/needle base
        self.J = [0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0, \
                  0,0,0,0,0,0,0]        #jacobian matrix
        self.cmd = [0,0]                #controller output
        self.get_logger().info('Experimental data will be saved at %s' %(self.filename))   

    #Get current entry_point
    def entry_point_callback(self, msg):
        self.entry_point = [msg.x, msg.y, msg.z]

    #Get current entry_point
    def target_callback(self, msg):
        self.target = [msg.x, msg.y, msg.z]

    #Get current Z
    def sensor_callback(self, msg):
        # From shape, get Z
        shape = msg.poses
        N = len(shape)
        self.Z = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z, \
            shape[N-1].orientation.w, shape[N-1].orientation.x, shape[N-1].orientation.y, shape[N-1].orientation.z])  #tip pose

    #Get current X
    def robot_callback(self, msg):
        robot = msg.pose
        self.X = [robot.position.x, robot.position.y, robot.position.z, \
            robot.orientation.w, robot.orientation.x, robot.orientation.y, robot.orientation.z]
        
    #Get current J
    def estimator_callback(self,msg):
        self.J = np.array(CvBridge().imgmsg_to_cv2(msg)).flatten()

    #Get current control output
    def control_callback(self,msg):
        self.cmd = [msg.point.x, msg.point.z]        
        
    #Save data do file
    def write_file_callback(self):
        now = self.get_clock().now().to_msg()
       
        data = [now.sec, now.nanosec, \
            self.entry_point[0], self.entry_point[1], self.entry_point[2], \
            self.target[0], self.target[1], self.target[2], \
            self.Z[0], self.Z[1], self.Z[2], self.Z[3], self.Z[4], self.Z[5], self.Z[6], \
            self.X[0], self.X[1], self.X[2], self.X[3], self.X[4], self.X[5], self.X[6], \
            self.J[0], self.J[1], self.J[2], self.J[3], self.J[4], self.J[5], self.J[6], \
            self.J[7], self.J[8], self.J[9], self.J[10], self.J[11], self.J[12], self.J[13], \
            self.J[14], self.J[15], self.J[16], self.J[17], self.J[18], self.J[19], self.J[20], \
            self.J[21], self.J[22], self.J[23], self.J[24], self.J[25], self.J[26], self.J[27], \
            self.J[28], self.J[29], self.J[30], self.J[31], self.J[32], self.J[33], self.J[34], \
            self.J[35], self.J[36], self.J[37], self.J[38], self.J[39], self.J[40], self.J[41], \
            self.J[42], self.J[43], self.J[44], self.J[45], self.J[46], self.J[47], self.J[48], \
            self.cmd[0], self.cmd[1]]
        
        with open(self.filename, 'a', newline='', encoding='UTF8') as f: # open the file in append mode
            writer = csv.writer(f) # create the csv writer
            writer.writerow(data)  # append a new row to the existing csv file     
        # self.get_logger().info('File saved')   

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
