import numpy as np
import rclpy

from cv_bridge import CvBridge
from cv_bridge.core import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Image
from transforms3d.euler import euler2quat


class EstimatorNode(Node):

    def __init__(self):
        super().__init__('estimator_node')

        #Topics from Stage node
        self.subscription_stage1 = self.create_subscription(PoseStamped, '/stage/state/pose', self.stage_callback, 10)
        self.subscription_stage1  # prevent unused variable warning

        self.subscription_stage2 = self.create_subscription(PoseStamped, '/stage/state/needle_pose', self.needle_pose_callback, 10)
        self.subscription_stage2  # prevent unused variable warning

        #Topics from UI node
        self.subscription_UI = self.create_subscription(PoseStamped, '/subject/state/skin_entry', self.entry_point_callback, 10)
        self.subscription_UI  # prevent unused variable warning

        #Topics from Sensor node
        self.subscription_sensor = self.create_subscription(PoseArray, '/needle/state/shape', self.needle_shape_callback, 10)
        self.subscription_sensor  # prevent unused variable warning

        #Published topics
        self.publisher_jacobian = self.create_publisher(Image, '/needle/state/jacobian', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_jacobian_callback)
        
        self.i = 0


    def stage_callback(self, msg):
        ##########################################
        # Get inputs Xpartial = [x_stage, z_stage]
        ##########################################
        stage = msg.pose.position
        #self.get_logger().info('Listening Stage - Stage position: x=%f, z=%f in %s frame'  % (stage.x, stage.z, msg.header.frame_id))

    def needle_pose_callback(self, msg):
        ##########################################
        # Get inputs Xpartial = [y_needle_depth, y_q]
        ##########################################
        needle = msg.pose
        #self.get_logger().info('Listening Stage - Needle pose: y=%f, q=%s in %s frame'  % (needle.position.y, needle.orientation, msg.header.frame_id))

    def entry_point_callback(self, msg):
        ##########################################
        # Define transform from stage to needle frame
        ##########################################
        entry_point = msg.pose.position
        #self.get_logger().info('Listening UI - Skin entry point: x=%f, y=%f, z=%f in %s frame'  % (entry_point.x, entry_point.y, \
        #    entry_point.z, msg.header.frame_id))

    def needle_shape_callback(self, msg):
        ##########################################
        # Get needle shape (FBG sensor measurements)
        shape = msg.poses
        #self.get_logger().info('Listening Sensor - Needle shape: %s in %s frame' % (shape, msg.header.frame_id))
        
        # From shape, get measured Z = [x_tip, y_tip, z_tip, q_tip] (Obs: for q, roll=pitch)
        N = len(shape)
        self.get_logger().info('N: %f ' % (N))
        if (N==1):
            q = [1, 0, 0, 0]
            #Quaternion(x=q[1], y=q[0], z=q[0], w=q[0])
        else:
            tip = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z])
            ptip = np.array([shape[N-2].position.x, shape[N-2].position.y, shape[N-2].position.z])
            dir = tip - ptip
            yaw = np.arctan2(dir[1], dir[0])
            pitch = np.arcsin(dir[2])
            roll = pitch
            q = euler2quat(yaw, roll, pitch, 'rzyx')
        Z = np.array([shape[N-1].position.x, shape[N-1].position.y, shape[N-1].position.z, q[0], q[1], q[2], q[3]])
        ##########################################
        self.get_logger().info('Z: %s in %s frame' % (np.array2string(Z), msg.header.frame_id))

    def timer_jacobian_callback(self):
        ##########################################
        # Calculate Z estimate (publish => optional)
        # Calculate Jacobian J
        J = np.ones((7,7))
        ##########################################
        msg = CvBridge().cv2_to_imgmsg(J)
        self.publisher_jacobian.publish(msg)
        #self.get_logger().info('Publish - Jacobian: %s' %  np.array2string(J))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    estimator_node = EstimatorNode()

    rclpy.spin(estimator_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
