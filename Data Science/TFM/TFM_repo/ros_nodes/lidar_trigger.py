import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger  # Adjust this if your service has a custom type

class ScanTriggerNode(Node):
    def __init__(self):
        super().__init__('scan_trigger_node')

        # Parameters
        self.declare_parameter('trigger_distance', 1.0)  # Set default trigger distance to 1.0 meters
        self.trigger_distance = self.get_parameter('trigger_distance').value

        # Subscription
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.scan_callback,
            10
        )


        self.safe_distance = 2.0

        # Service Client
        self.service_client = self.create_client(Trigger, '/lane_change')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /service to become available...')

        self.get_logger().info('Scan Trigger Node initialized.')

    def scan_callback(self, msg: LaserScan):
        """
        Callback to process LaserScan data and trigger a service if an object is within the specified distance.
        Only considers scans in the front-facing direction.
        """
        
        ranges = msg.ranges
        min_distance = min(ranges)
        

        if min_distance < self.safe_distance:
            self.trigger_service()

    def trigger_service(self):
        """
        Calls the /service if the trigger condition is met.
        """
        self.get_logger().info('Object detected within trigger distance. Triggering service call...')

        # Create an empty request
        request = Trigger.Request()

        # Call the service and handle the response
        future = self.service_client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """
        Callback to handle the response from the service call.
        """
        try:
            response = future.result()
            self.get_logger().info(f'Service call successful: {response.success}, Message: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ScanTriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
