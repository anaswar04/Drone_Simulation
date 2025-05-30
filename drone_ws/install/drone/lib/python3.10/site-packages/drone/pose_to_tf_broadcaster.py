#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class PoseToTFBroadcaster(Node):

    def __init__(self):
        super().__init__('pose_to_tf_broadcaster')

        # Fix QoS issue: match MAVROS' best-effort policy
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos
        )

        self.br = TransformBroadcaster(self)
        self.get_logger().info('TF Broadcaster Initialized.')

    def pose_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'  # world frame
        t.child_frame_id = 'base_link'  # drone frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

