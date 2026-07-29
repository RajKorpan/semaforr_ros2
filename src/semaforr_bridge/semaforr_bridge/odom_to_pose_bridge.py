import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class OdomToPoseBridge(Node):
    def __init__(self):
        super().__init__('odom_to_pose_bridge')

        self.declare_parameter(
            'odometry_topic',
            '/mobile_base_controller/odom',
        )
        self.declare_parameter('pose_topic', '/pose')

        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter('odometry_topic').value,
            self.odom_callback,
            10,
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            10,
        )

    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        self.publisher.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
