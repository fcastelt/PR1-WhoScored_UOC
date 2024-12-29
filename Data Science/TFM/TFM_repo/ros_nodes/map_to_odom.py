import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile
import math

class TransformNode(Node):
    def __init__(self):
        super().__init__('odom_to_map_node')
        
        # Parameters for frame IDs
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_link_frame_id', 'base_link')

        self.odom_frame_id_ = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.map_frame_id_ = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.base_link_frame_id_ = self.get_parameter('base_link_frame_id').get_parameter_value().string_value

        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscription to Odometry topic to get pose estimates
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )
        
        # Timer to publish the map-to-odom transform every 3 seconds
       # self.transform_timer = self.create_timer(0.1, self.publish_static_transform)

        # Variable to hold the latest odometry pose
        self.latest_odometry = None

    def odom_callback(self, msg):
        # Continuously update the latest odometry data
        self.latest_odometry = msg
        self.publish_static_transform()

    def publish_static_transform(self):
        if self.latest_odometry is None:
            self.get_logger().warn("No odometry data received yet.")
            return

        # Create the TransformStamped message
        static_transform = TransformStamped()
        static_transform.header.stamp = self.latest_odometry.header.stamp  # Use the latest timestamp from odometry
        static_transform.header.frame_id = self.map_frame_id_
        static_transform.child_frame_id = self.odom_frame_id_

        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z =  0.0 
        static_transform.transform.rotation.w =  1.0  

        # Publish the static transform
        self.tf_broadcaster.sendTransform(static_transform)
        #self.get_logger().info(f'Transform is quaternion: {static_transform.transform.rotation.x}, {static_transform.transform.rotation.y}, {static_transform.transform.rotation.z}, {static_transform.transform.rotation.w}')
        

def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
