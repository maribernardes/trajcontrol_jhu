import rclpy
import ament_index_python 
import csv

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SaveFile(Node):

    def __init__(self):
        super().__init__('save_file')

        
        #Declare node parameters
        self.declare_parameter('filename', 'my_data') #Name of file where data values are saved
        package_path = str(ament_index_python.get_package_share_path('trajcontrol')) 
        self.filename = package_path + '/../../../../src/trajcontrol/data/' + self.get_parameter('filename').get_parameter_value().string_value + '.csv' #String with full path to file

        #Topics from UI node
        self.subscription_UI = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_UI  # prevent unused variable warning

        #Topics from robot node
        self.subscription_robot = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.base_pose_callback, 10)
        self.subscription_robot  # prevent unused variable warning

        #Published topics
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.write_file_callback)

        #Array of data
        header = ['Timestamp sec', 'Timestamp nanosec', \
            'Entry_point x', 'Entry_point y', 'Entry_point z', \
            'Base_posistion x', 'Base_posistion y', 'Base_posistion z', \
            'Base_orientation qx', 'Base_orientation qy', 'Base_orientation qz', 'Base_orientation qw']
        
        with open(self.filename, 'w', newline='', encoding='UTF8') as f: # open the file in the write mode
            writer = csv.writer(f)  # create the csv writer
            writer.writerow(header) # write a row to the csv file

        #Last data received
        self.entry_point = PoseStamped().pose.position
        self.base_pose = PoseStamped().pose

    # Get current entry_point
    def entry_point_callback(self, msg):
        self.entry_point = msg.pose.position

    # Get current base_pose
    def base_pose_callback(self, msg):
        self.base_pose = msg.pose

    # Save data do file
    def write_file_callback(self):
        now = self.get_clock().now().to_msg()
        data = [now.sec, now.nanosec, self.entry_point.x, self.entry_point.y, self.entry_point.z, \
            self.base_pose.position.x, self.base_pose.position.y, self.base_pose.position.z, \
                self.base_pose.orientation.x, self.base_pose.orientation.y, self.base_pose.orientation.z, self.base_pose.orientation.w]
        
        with open(self.filename, 'a', newline='', encoding='UTF8') as f: # open the file in append mode
            writer = csv.writer(f) # create the csv writer
            writer.writerow(data)  # append a new row to the existing csv file
        self.get_logger().info('saved file')        

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
