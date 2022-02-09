import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VirtualUI(Node):

    def __init__(self):
        super().__init__('virtual_UI')

        #Published topics
        self.publisher_skin = self.create_publisher(PoseStamped, '/subject/state/skin_entry', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_skin_entry_callback)

        self.publisher_target = self.create_publisher(PoseStamped, '/subject/state/target', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_target_callback)

        self.i = 0


    def timer_skin_entry_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"

        msg.pose.position.x = 1.0 + self.i
        msg.pose.position.y = 2.0 + self.i
        msg.pose.position.z = 3.0 + self.i

        self.publisher_skin.publish(msg)
        self.get_logger().info('Publish - Skin entry point: x=%f, y=%f, z=%f in %s frame'  % (msg.pose.position.x, \
            msg.pose.position.y, msg.pose.position.z, msg.header.frame_id))
        self.i += 1

    def timer_target_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stage"

        msg.pose.position.x = 1.0 + self.i
        msg.pose.position.y = 2.0 + self.i
        msg.pose.position.z = 3.0 + self.i

        self.publisher_target.publish(msg)
        self.get_logger().info('Publish - Target point: x=%f, y=%f, z=%f in %s frame' % (msg.pose.position.x, \
            msg.pose.position.y, msg.pose.position.z, msg.header.frame_id))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    virtual_UI = VirtualUI()

    rclpy.spin(virtual_UI)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_UI.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()