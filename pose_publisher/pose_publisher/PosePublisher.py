import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'pose_stamped', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        pose_stamped_msg = PoseStamped()

        # ヘッダー (座標フレームとタイムスタンプ)
        pose_stamped_msg.header = Header()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = "map"  # 座標フレームを指定

        # 位置を指定 (x, y, z)
        pose_stamped_msg.pose.position = Point(x=1.0, y=2.0, z=0.0)

        # 姿勢を指定 (Quaternion)
        pose_stamped_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.publisher_.publish(pose_stamped_msg)
        self.get_logger().info('Publishing pose stamped: %s' % pose_stamped_msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
