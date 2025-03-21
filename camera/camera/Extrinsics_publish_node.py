import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_matrix

class ExtrinsicPublisher(Node):
    def __init__(self):
        super().__init__('extrinsic_publisher')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/camera/Extrinsics',
            self.pose_callback,
            1
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("ExtrinsicPublisher Node Started")

    def pose_callback(self, msg):
        self.transform = TransformStamped()
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = "tr_base"
        self.transform.child_frame_id = "tr_marker"
        
        self.transform.transform.translation.x = msg.pose.position.x
        self.transform.transform.translation.y = msg.pose.position.y
        self.transform.transform.translation.z = msg.pose.position.z

        self.transform.transform.rotation.x = msg.pose.orientation.x
        self.transform.transform.rotation.y = msg.pose.orientation.y
        self.transform.transform.rotation.z = msg.pose.orientation.z
        self.transform.transform.rotation.w = msg.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(self.transform)
        self.get_logger().info("Published TF transform from camera_frame to aruco_marker")


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()