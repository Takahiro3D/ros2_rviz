import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

class PoseArrayPublisher(Node):
    def __init__(self):
        super().__init__('pose_array_with_index_publisher')
        self.pose_publisher_ = self.create_publisher(PoseArray, 'pose_array', 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish_pose_and_index)

    def publish_pose_and_index(self):
        pose_array_msg = PoseArray()
        marker_array_msg = MarkerArray()

        # ヘッダー (座標フレームとタイムスタンプ)
        pose_array_msg.header = Header()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "map"  # 座標フレームを指定

        # 複数のPoseを作成
        poses = []
        for i in range(3):
            pose = Pose()
            pose.position = Point(x=1.0 + i, y=2.0 + i, z=0.0)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            poses.append(pose)

            # 各姿勢に対応するインデックスを表示するためのMarkerを作成
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "index_marker"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = pose.position
            marker.pose.position.z += 0.5  # テキストを少し上に表示
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.text = str(i)  # インデックスをテキストとして設定

            marker_array_msg.markers.append(marker)

        # PoseArrayに追加
        pose_array_msg.poses = poses

        # パブリッシュ
        self.pose_publisher_.publish(pose_array_msg)
        self.marker_publisher_.publish(marker_array_msg)

        self.get_logger().info('Publishing pose array and index markers')

def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
