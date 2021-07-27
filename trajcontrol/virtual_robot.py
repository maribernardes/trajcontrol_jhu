import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from transforms3d.euler import euler2quat

class VirtualRobot(Node):

    def __init__(self):
        super().__init__('virtual_robot')

        #Published topics
        self.publisher_pose = self.create_publisher(PoseStamped, '/stage/state/pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_stage_callback)

        self.publisher_needle_pose = self.create_publisher(PoseStamped, '/stage/state/needle_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_needlepose_callback)

        self.publisher_needle = self.create_publisher(PoseStamped, '/needle/state/pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_needle_callback)

        self.i = 0


    def timer_stage_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"

        msg.pose.position.x = 1.0 + self.i
        msg.pose.position.z = 2.0 + self.i

        self.publisher_pose.publish(msg)
        self.get_logger().info('Publish - Stage position: x=%f z=%f in %s frame'  % (msg.pose.position.x, msg.pose.position.z, msg.header.frame_id))
        self.i += 1

    def timer_needlepose_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"

        msg.pose.position.z = 3.0 + self.i
        q = euler2quat(0, 0.1745+0.1*self.i, 0, 'rzyx')
        msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.publisher_needle_pose.publish(msg)
        self.get_logger().info('Publish - Needle pose: y=%f, q=[%f, %f, %f, %f] in %s frame'  % (msg.pose.position.y, msg.pose.orientation.x, \
             msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, msg.header.frame_id))
        self.i += 1

    def timer_needle_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "needle"

        msg.pose.position.z = 1.0 + self.i
        q = euler2quat(0, 0.1745+0.15*self.i, 0, 'rzyx')
        msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.publisher_needle.publish(msg)
        self.get_logger().info('Publish - Needle pose: y=%f, q=[%f, %f, %f, %f] in %s frame'  % (msg.pose.position.y, msg.pose.orientation.x, \
             msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w, msg.header.frame_id))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    virtual_robot = VirtualRobot()

    rclpy.spin(virtual_robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()