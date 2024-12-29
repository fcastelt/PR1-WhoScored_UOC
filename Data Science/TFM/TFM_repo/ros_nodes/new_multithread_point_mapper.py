import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, Float32MultiArray, Int8MultiArray
import tf2_ros
from tf2_ros import TransformListener, Buffer
import tf_transformations
from rclpy.qos import QoSProfile, DurabilityPolicy
import time
import csv
import threading
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from chimera_custom_interfaces.msg import PoseWithLanePoints



class DynamicMapNode(Node):
    def __init__(self):
        super().__init__('dynamic_map_node')

        # Define Callback Groups for separate threads
        self.lane_callback_group = ReentrantCallbackGroup()
        self.map_update_callback_group = MutuallyExclusiveCallbackGroup()

        # Occupancy grid parameter
        self.left_lane_updated = False
        self.right_lane_updated = False
        self.neighbour_left_lane_updated = False
        self.neighbour_right_lane_updated = False

        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_resolution = 0.02  # Resolution in meters per cell
        self.map_width = int(15.0 / self.map_resolution)  # 15 meters
        self.map_height = int(10.0 / self.map_resolution)  # 10 meters
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        # Publisher for the occupancy grid
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_profile)

        # TF listener to track the car's position and orientation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to update the map

        # Subscriptions for lane points
        self.lane_points_sub = self.create_subscription(PoseWithLanePoints, "lane_points", self.lane_points_callback, 10, callback_group=self.lane_callback_group)


        self.lane_info_sub = self.create_subscription(Int8MultiArray, "lane_info", self.lane_info_callback, 10)

        self.right_lane_points = []
        self.left_lane_points = []
        self.neighbour_right_lane_points = []
        self.neighbour_left_lane_points = []
        self.center_lane_points = []
        # Camera offsets
        self.camera_offset_x = 1.65 # FORWARD
        self.camera_offset_y = 1.4775  # lateral

        self.flag_lane_found = 0

        self.car_x = None
        self.car_y = None
        self.car_yaw = None

        self.new_pose_received = False
        self.new_lane_points_received = False


    def lane_info_callback(self, msg):
        self.flag_lane_found = msg.data[0]
        self.course = msg.data[1]
        self.flag_dashed = msg.data[2]
        # self.update_car_position()
        # self.update_map()

    def lane_points_callback(self, msg):
        # Unpack the Pose
        try:
            car_x = msg.pose.position.x
            car_y = msg.pose.position.y
            print(msg.pose.orientation)
            rotation_quat = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            _, _, car_yaw = tf_transformations.euler_from_quaternion(rotation_quat)

            # Unpack the lane points
            left_lane_points = np.array(msg.left_lane_points.data).reshape(-1, 2) * -1
            right_lane_points = np.array(msg.right_lane_points.data).reshape(-1, 2) * -1
            center_lane_points = np.array(msg.center_lane_points.data).reshape(-1, 2) * -1

            # Unpack neighbor lane points
            neighbour_left_lane_points = np.array(msg.neighbour_left_lane_points.data).reshape(-1, 2) * -1
            neighbour_center_lane_points = np.array(msg.neighbour_center_lane_points.data).reshape(-1, 2) * -1
            neighbour_right_lane_points = np.array(msg.neighbour_right_lane_points.data).reshape(-1, 2) * -1

        # Call the function to update the map (or perform other processing)
            self.update_map(car_x, car_y, car_yaw, left_lane_points[:7], right_lane_points[:7], neighbour_left_lane_points[:7], neighbour_right_lane_points[:7])
            
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")


    def update_map(self, car_x, car_y, car_yaw, left_lane_points, right_lane_points, neighbour_left_lane_points, neighbour_right_lane_points):
        if self.flag_lane_found != 1:
            self.get_logger().info("Lane not found, stopping map update.")
            return

        try:

            if self.flag_dashed == -1:
                self.plot_and_fill_lanes(car_x, car_y, car_yaw, left_lane_points, neighbour_right_lane_points)
            elif self.flag_dashed == 0:
                self.plot_and_fill_lanes(car_x, car_y, car_yaw, left_lane_points, right_lane_points)
            elif self.flag_dashed == 1:
                self.plot_and_fill_lanes(car_x, car_y, car_yaw, neighbour_left_lane_points, right_lane_points)

            #self.publish_map()
        except Exception as e:
            self.get_logger().warn(f"Error updating map: {e}")
        



    def plot_and_fill_lanes(self, car_x, car_y, car_yaw, left_points, right_points, padding=1):
        """
        Plot polylines for left and right lanes on the occupancy grid and fill the space between them.

        Parameters:
        - car_x, car_y: Robot's position in the map.
        - car_yaw: Robot's yaw angle in radians.
        - left_points, right_points: Lane points in the camera frame for left and right lanes (pixel coordinates scaled to meters).
        - padding: Extra cells to mark around the interpolated points as free space.
        """
        # Helper functions
        def interpolate_lane(points):
            """
            Interpolates points between consecutive points in a lane.
            """
            interpolated = []
            for i in range(len(points) - 1):
                start, end = points[i], points[i + 1]
                t = np.linspace(0, 1, num_interpolation)
                x_range = start[1] + t * (end[1] - start[1])
                y_range = start[0] + t * (end[0] - start[0])
                interpolated.append(np.stack((y_range, x_range), axis=-1))
            return np.vstack(interpolated)

        def transform_points(points):
            """
            Apply rotation and translation to points in the car's frame to map coordinates.
            """
            rotated = np.empty_like(points)
            rotated[:, 0] = car_x + points[:, 1] * cos_yaw - points[:, 0] * sin_yaw
            rotated[:, 1] = car_y + points[:, 1] * sin_yaw + points[:, 0] * cos_yaw
            return rotated

        def to_grid_coords(points):
            """
            Convert world coordinates to occupancy grid coordinates.
            """
            grid_x = (points[:, 0] / self.map_resolution + self.map_width // 2).astype(int)
            grid_y = (points[:, 1] / self.map_resolution + self.map_height // 2).astype(int)
            return grid_x, grid_y

        def fill_space_between_lanes(left_transformed, right_transformed):
            """
            Fill the space between the left and right lanes on the occupancy grid.
            """
            def interpolate_between_points(point_a, point_b):
                x_range = np.linspace(point_a[0], point_b[0], num_fill_points)[1:-1]
                y_range = np.linspace(point_a[1], point_b[1], num_fill_points)[1:-1]
                return np.stack((x_range, y_range), axis=-1)

            # Generate interpolated points for the space between lanes
            all_fill_points = []
            for (lx, ly), (rx, ry) in zip(left_transformed, right_transformed):
                fill_points = interpolate_between_points((lx, ly), (rx, ry))
                all_fill_points.append(fill_points)
            all_fill_points = np.vstack(all_fill_points)

            # Map to grid coordinates and apply padding
            fill_grid_x, fill_grid_y = to_grid_coords(all_fill_points)
            dx, dy = np.meshgrid(range(-padding, padding + 1), range(-padding, padding + 1))
            dx, dy = dx.ravel(), dy.ravel()

            adj_x = fill_grid_x[:, None] + dx
            adj_y = fill_grid_y[:, None] + dy
            valid_mask = (0 <= adj_x) & (adj_x < self.map_width) & (0 <= adj_y) & (adj_y < self.map_height)
            adj_x = adj_x[valid_mask]
            adj_y = adj_y[valid_mask]

            self.occupancy_grid[adj_y, adj_x] = 0  # Mark as free space

        # Main logic
        # Convert points to NumPy arrays and adjust for camera offsets
        left_points = np.array(left_points)
        right_points = np.array(right_points)
        left_points[:, 0] += self.camera_offset_y
        left_points[:, 1] += self.camera_offset_x
        right_points[:, 0] += self.camera_offset_y
        right_points[:, 1] += self.camera_offset_x

        # Interpolate points
        num_interpolation = 7
        left_interpolated = interpolate_lane(left_points)
        right_interpolated = interpolate_lane(right_points)

        # Apply rotation and translation
        sin_yaw, cos_yaw = np.sin(car_yaw), np.cos(car_yaw)
        left_transformed = transform_points(left_interpolated)
        right_transformed = transform_points(right_interpolated)

        # Map left and right lanes to the grid
        left_grid_x, left_grid_y = to_grid_coords(left_transformed)
        right_grid_x, right_grid_y = to_grid_coords(right_transformed)

        # Fill space between lanes
        num_fill_points = 13
        fill_space_between_lanes(left_transformed, right_transformed)

        # Mark the lanes on the occupancy grid
        # Combine left and right grid coordinates
        all_grid_x = np.concatenate([left_grid_x, right_grid_x])
        all_grid_y = np.concatenate([left_grid_y, right_grid_y])

        # Create a valid mask for both left and right lane points at once
        valid_mask = (0 <= all_grid_x) & (all_grid_x < self.map_width) & (0 <= all_grid_y) & (all_grid_y < self.map_height)

        # Apply the mask to select valid indices
        valid_x = all_grid_x[valid_mask]
        valid_y = all_grid_y[valid_mask]

        # Mark the valid positions as occupied in one operation
        self.occupancy_grid[valid_y, valid_x] = 100  # 100 indicates occupied

        self.publish_map()
        

    def publish_map(self):
        """Publish the updated map as an OccupancyGrid message."""
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = "map"
        occupancy_grid_msg.info.resolution = self.map_resolution
        occupancy_grid_msg.info.width = self.map_width
        occupancy_grid_msg.info.height = self.map_height
        occupancy_grid_msg.info.origin.position.x = -self.map_width // 2 * self.map_resolution
        occupancy_grid_msg.info.origin.position.y = -self.map_height // 2 * self.map_resolution
        occupancy_grid_msg.data = self.occupancy_grid.flatten().tolist()
        self.map_pub.publish(occupancy_grid_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create node instance
    dynamic_map_node = DynamicMapNode()
    
    # Multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=2)  
    executor.add_node(dynamic_map_node)
    
    try:
        # Run the executor
        executor.spin()
    finally:
        dynamic_map_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
