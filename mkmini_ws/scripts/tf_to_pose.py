#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped

class TFToPose(Node):
    def __init__(self):
        super().__init__('tf_to_pose')
        self.pub = self.create_publisher(PoseStamped, '/vehicle_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.02, self.tick)  # 50Hz

    def tick(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
            self.pub.publish(msg)
        except Exception:
            pass

def main():
    rclpy.init()
    rclpy.spin(TFToPose())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
