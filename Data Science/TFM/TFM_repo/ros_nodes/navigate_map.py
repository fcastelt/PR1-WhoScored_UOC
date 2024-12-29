import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from tf2_ros         import TransformListener, Buffer
import tf_transformations
import numpy as np
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
import time
from chimera_custom_interfaces.msg import PoseWithLanePoints
from geometry_msgs.msg import Quaternion
import sys
class NavigateMap(Node):
    def __init__(self):
        super().__init__('navigate_map')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.center_lane_point_number = 5
        self.neighbour_lane_point_number = 9
        self.update_nav_to_pose_frequency = 0.1

        # Initialize TF2 for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.change_event_service = self.create_service(
            Trigger,
            'lane_change',
            self.handle_change_event)


        self.lane_points_sub = self.create_subscription(PoseWithLanePoints, "lane_points", self.lane_points_callback, 10)
        
        self.quaternion_sub = self.create_subscription(Quaternion, "quaternion", self.quaternion_callback, 10)

        self.center_lane_points = []
        self.neighbour_center_lane_point = []
        self.last_seen_point = None  # Store the last valid lane point

        self.camera_offset_x = 1.65  # Adjust according to camera position
        self.camera_offset_y = 1.45  # Adjust according to camera position

        self.map_resolution = 0.02  # Resolution in meters per cell
        self.map_width = int(15.0 / self.map_resolution)  # 15 meters
        self.map_height = int(10.0 / self.map_resolution)

        self.quaternion = None

        #self.timer1_group = MutuallyExclusiveCallbackGroup()
        #self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.navigate_loop, 10)
        #self.switch_goal_group = MutuallyExclusiveCallbackGroup()

        #self.timer = self.create_timer(2, self.start_navigation)
       
        
        self.timer = self.create_timer(self.update_nav_to_pose_frequency, self.navigate_loop)
       # self.switch_neighbour_lane = self.create_timer(6, self.change_course, callback_group=self.switch_goal_group)
        
        self.switch_to_neighbour_lane = False
        self.switching = False  

        self.goal_status_success = False

        self.publish_once = False

        self.goal_timeout = 6.0  # 6 seconds for neighbor goal timeout
        self.goal_start_time = None 
        #self.timer = self.create_timer(2.0, self.start_navigation)
    
    def quaternion_callback(self, msg):
        self.quaternion = msg
        #self.get_logger().info("Quaternion value is " + str(self.quaternion))
    
    def start_navigation(self):
        """
        Method called after initialization to invoke navigate_loop once.
        """
        self.get_logger().info("Starting navigation loop after initialization.")
        self.navigate_loop()
        self.timer.cancel()  


    def handle_change_event(self, request, response):
        self.get_logger().info("Change event triggered!")
        self.switch_to_neighbour_lane = True
        response.success = True
        response.message = "Event processed successfully"
        return response

    def change_course(self):
        self.switch_to_neighbour_lane = True
        self.get_logger().info("Switching to neighbour lane")

    def lane_info_callback(self, msg):
        """Callback to process lane info."""
        self.flag_lane_found = msg.data[0]
        self.course = msg.data[1]
        self.flag_dashed = msg.data[2]
    
    def lane_points_callback(self, msg):
        # Unpack the Pose
        try:
            self.car_x = msg.pose.position.x
            self.car_y = msg.pose.position.y
            #print(msg.pose.orientation)
            rotation_quat = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            _, _, self.car_yaw = tf_transformations.euler_from_quaternion(rotation_quat)
            self.center_lane_points = np.array(msg.center_lane_points.data).reshape(-1, 2) * -1
            self.neighbour_center_lane_points = np.array(msg.neighbour_center_lane_points.data).reshape(-1, 2) * -1
            #
            #self.navigate_loop()
            
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def transform_lane_point(self, x, y):

        """Transform a single lane point based on the robot's current pose."""
        try:
            
            car_x = self.car_x
            car_y = self.car_y
            car_yaw = self.car_yaw

            # Adjust point for camera offset in the robot's local frame
            adjusted_x = x + self.camera_offset_x
            adjusted_y = y + self.camera_offset_y

            # Rotate the point based on the robot's orientation
            rotated_x = adjusted_x * np.cos(car_yaw) - adjusted_y * np.sin(car_yaw)
            rotated_y = adjusted_x * np.sin(car_yaw) + adjusted_y * np.cos(car_yaw)

            # Translate the point into the global (map) frame
            global_x = car_x + rotated_x
            global_y = car_y + rotated_y

            return global_x, global_y, car_yaw
        except Exception as e:
            self.get_logger().error(f"Error transforming lane point: {e}")
            return None
    
    def create_pose_from_points(self, points, starting_point):
        """Create a PoseStamped message from the given points."""
        x_point = points[starting_point][1]
        y_point = points[starting_point][0]

        x_transformed, y_transformed, car_yaw = self.transform_lane_point(x_point, y_point)

        # Create the goal Pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'  # Assuming the map frame
        target_pose.header.stamp = self.get_clock().now().to_msg()

        target_pose.pose.position.x = x_transformed
        target_pose.pose.position.y = y_transformed
        target_pose.pose.orientation = self.quaternion  # Adjust orientation as needed

        # Update the last seen point
        return target_pose

    def navigate_loop(self, msg=None):
        try:
            # if self.publish_once == True:
            #      return
            if self.switching:
                # Check timeout for neighbor lane goal
                if self.switch_to_neighbour_lane and (time.time() - self.goal_start_time > self.goal_timeout):
                    self.get_logger().warn("Neighbor lane goal timed out. Returning to center lane.")
                    self.switch_to_neighbour_lane = False
                    self.switching = False
                return

            if not self.switch_to_neighbour_lane:
                # Navigate to the center lane
                lane_points = self.center_lane_points
                target_pose = self.create_pose_from_points(lane_points, self.center_lane_point_number)
                self.send_goal(target_pose, neighbour_lane=False)
                self.get_logger().info("target pose sended is " + str(target_pose))
            else:
                # Navigate to the neighbor lane
                lane_points = self.neighbour_center_lane_points
                target_pose = self.create_pose_from_points(lane_points, self.neighbour_lane_point_number)
                self.send_goal(target_pose, neighbour_lane=True)
                self.switching = True
        except Exception as e:
            self.get_logger().error(f"Error in navigate_loop: {e}")



    def send_goal(self, pose, neighbour_lane=False):
        """Send the goal to the NavigateToPose action server."""
        if not self._action_client.server_is_ready():
            self.get_logger().info('Action server not ready')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        #self.get_logger().info('Sending goal...')
        if neighbour_lane == False:
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.publish_once = True
        else:
            self.goal_start_time = time.time()
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.neighbour_feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_neighbour_response_callback)
        
    def goal_neighbour_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        #self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_neighbour_result_callback)

    def get_neighbour_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Neighbor lane goal reached successfully.')
            self.switch_to_neighbour_lane = False
            self.switching = False
            self.navigate_loop()
        else:
            self.get_logger().warn('Neighbor lane goal failed or canceled.')
            self.switch_to_neighbour_lane = False
            self.switching = False
            
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        #self.get_logger().info('Goal accepted.')
        #self._get_result_future = goal_handle.get_result_async()
        #self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Goal reached successfully.')
                #sys.exit(0)
                # self.switch_to_neighbour_lane = False
                # self.switching = False
                # self.navigate_loop()
        #self.get_logger().info(f'Result received: {result.result}')

    def feedback_callback(self, feedback_msg):
        """
        Feedback callback to calculate the distance between the car and the sent goal pose.
        Sends another goal when close enough.
        """

        try:
            #return None
            #self.get_logger().info("")
            # Extract the current robot position from TF
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            car_x = transform.transform.translation.x
            car_y = transform.transform.translation.y

            # Get the target pose from feedback
            goal_pose = feedback_msg.feedback.current_pose.pose
            goal_x = goal_pose.position.x
            goal_y = goal_pose.position.y

            # Calculate the Euclidean distance to the target
            distance = np.sqrt((goal_x - car_x) ** 2 + (goal_y - car_y) ** 2)
            self.get_logger().info(f"Distance to goal: {distance:.2f} meters")

            # Threshold to determine when close enough to send the next pose
            threshold_distance = 0.001  # Adjust threshold as needed
            if distance < threshold_distance:
                self.get_logger().info("Close enough to the goal. Sending the next pose...")
                #Trigger navigation to the next pose
                self.navigate_loop()

        except Exception as e:
            self.get_logger().error(f"Error in feedback_callback: {e}")

    def neighbour_feedback_callback(self, feedback_msg):
        """
        Feedback callback to calculate the distance between the car and the sent goal pose.
        Sends another goal when close enough.
        """
        #return None
        try:
            print(feedback_msg.feedback.distance_remaining)
            # Extract the current robot position from TF
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            car_x = transform.transform.translation.x
            car_y = transform.transform.translation.y

            # Get the target pose from feedback
            goal_pose = feedback_msg.feedback.current_pose.pose
            goal_x = goal_pose.position.x
            goal_y = goal_pose.position.y

            # Calculate the Euclidean distance to the target
            distance = np.sqrt((goal_x - car_x) ** 2 + (goal_y - car_y) ** 2)
            #self.get_logger().info(f"Distance to goal: {distance:.2f} meters")
            threshold_distance = 0.001

            # Threshold to determine when close enough to send the next pose
            # if distance < threshold_distance:
            #    # self.get_logger().info("Close enough to the goal. Sending the next pose...")
            #     # Trigger navigation to the next pose
            #     self.switch_to_neighbour_lane = False
            #     self.switching = False
            #     self.navigate_loop()
            #     sys.exit(0)

        except Exception as e:
            self.get_logger().error(f"Error in feedback_callback: {e}")


    
def main(args=None):
    rclpy.init(args=args)
    node = NavigateMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()