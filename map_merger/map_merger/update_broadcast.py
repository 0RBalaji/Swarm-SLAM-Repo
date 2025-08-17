import rclpy
import threading
import math
import numpy as np
from copy import deepcopy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class MapUpdateBroadcast(Node):
    def __init__(self):
        super().__init__('map_update_broadcast')

        # Declare parameters with proper types
        self.declare_parameter('namespace', '', ParameterDescriptor(description='Robot namespace'))
        self.declare_parameter('change_buffer', 2, ParameterDescriptor(description='Bounding box buffer size'))
        # self.declare_parameter('ignore_unexplored', True, ParameterDescriptor(description='Filter unexplored areas'))
        # self.declare_parameter('max_delta_size', 256, ParameterDescriptor(description='Max allowed cropped dimension'))

        # Retrieve parameters with type casting
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.buffer_size = self.get_parameter('change_buffer').get_parameter_value().integer_value
        # self.ignore_unexplored = self.get_parameter('ignore_unexplored').get_parameter_value().bool_value
        # self.max_delta_size = self.get_parameter('max_delta_size').get_parameter_value().integer_value

        if not self.namespace:
            self.get_logger().fatal("Namespace parameter required. Exiting.")
            rclpy.shutdown()
            return

        # Configure QoS profile
        qos = QoSProfile(
            depth=50,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Initialize communication
        self.delta_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.namespace}/map_changes',
            qos
        )
        
        # self.map_client = self.create_client(GetMap, f'/{self.namespace}/map_server/get_map')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/map',
            self.map_callback,
            qos
        )

        # State management
        self.previous_map = None
        self.current_map = None
        # self.map_lock = threading.Lock()

         # Timer to periodically request the map
        # self.create_timer(5.0, self.request_map)  # Calls the service every 2 seconds

    # def request_map(self):
    #     """Request map using GetMap service"""
    #     if not self.map_client.wait_for_service(timeout_sec=2.0):
    #         self.get_logger().warn("Map service not available")
    #         return

    #     req = GetMap.Request()
    #     future = self.map_client.call_async(req)
    #     future.add_done_callback(self.process_map_response)

    # def process_map_response(self, future):
    #     """Process the received map from GetMap service"""
    #     try:
    #         response = future.result()
    #         self.map_callback(response.map)
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {str(e)}")

    def small_game(self, prev_map):
        if prev_map:
            self.delta_pub.publish(prev_map)
            # self.publish_changes()
        return

    def map_callback(self, msg):
        """Process incoming map updates"""
        # with self.map_lock:
        # if not self.validate_map(msg):
        #     return

        if not self.previous_map:
            self.previous_map = msg
            self.get_logger().info("Initial map stored")
            self.small_game(self.previous_map)
            return

        try:
            aligned_prev, aligned_new = self.align_maps(self.previous_map, msg)
            delta = self.detect_changes(aligned_prev, aligned_new)
            
            if not np.any(delta != -1):
                self.get_logger().debug("No significant changes detected")
                return

            bbox = self.calculate_bounding_box(delta)
            if not bbox:
                return

            cropped_delta = self.crop_delta(delta, bbox)
            # if self.validate_crop_size(cropped_delta):
            self.publish_change(cropped_delta, msg.info, msg.header, bbox)

            self.previous_map = msg

        except Exception as e:
            self.get_logger().error(f"Map processing failed: {str(e)}")

    # def validate_map(self, map_msg):
    #     """Validate map message integrity"""
    #     if not map_msg.data:
    #         self.get_logger().warn("Empty map received")
    #         return False
    #     if map_msg.info.resolution <= 0:
    #         self.get_logger().error("Invalid map resolution")
    #         return False
    #     return True

    def align_maps(self, prev_map, new_map):
        """Align maps to common coordinate system"""
        if prev_map.info.resolution != new_map.info.resolution:
            raise ValueError("Map resolution mismatch")

        # Calculate unified coordinate bounds
        prev_origin = prev_map.info.origin.position
        new_origin = new_map.info.origin.position
        res = prev_map.info.resolution

        min_x = min(prev_origin.x, new_origin.x)
        min_y = min(prev_origin.y, new_origin.y)
        max_x = max(
            prev_origin.x + prev_map.info.width * res,
            new_origin.x + new_map.info.width * res
        )
        max_y = max(
            prev_origin.y + prev_map.info.height * res,
            new_origin.y + new_map.info.height * res
        )

        # Create aligned grids
        width = math.ceil((max_x - min_x) / res)
        height = math.ceil((max_y - min_y) / res)
        
        aligned_prev = np.full((height, width), -1, dtype=np.int8)
        aligned_new = np.full((height, width), -1, dtype=np.int8)

        # Copy map data with proper coordinate transformation
        self.copy_map_data(prev_map, aligned_prev, min_x, min_y)
        self.copy_map_data(new_map, aligned_new, min_x, min_y)

        return aligned_prev, aligned_new

    def copy_map_data(self, source_map, dest_grid, global_min_x, global_min_y):
        """Copy map data to aligned grid"""
        res = source_map.info.resolution
        origin_x = source_map.info.origin.position.x
        origin_y = source_map.info.origin.position.y

        # Calculate grid positions
        start_col = int((origin_x - global_min_x) / res)
        start_row = int((origin_y - global_min_y) / res)
        end_col = start_col + source_map.info.width
        end_row = start_row + source_map.info.height

        # Calculate valid regions with boundary checks
        col_start = max(0, start_col)
        col_end = min(dest_grid.shape[1], end_col)
        row_start = max(0, start_row)
        row_end = min(dest_grid.shape[0], end_row)

        if col_end <= col_start or row_end <= row_start:
            return

        # Reshape source data
        source_data = np.array(source_map.data, dtype=np.int8).reshape(
            (source_map.info.height, source_map.info.width)
        )
        
        # Calculate source region indices
        src_row_start = row_start - start_row
        src_row_end = src_row_start + (row_end - row_start)
        src_col_start = col_start - start_col
        src_col_end = src_col_start + (col_end - col_start)

        # Perform array slicing
        dest_grid[row_start:row_end, col_start:col_end] = \
            source_data[src_row_start:src_row_end, src_col_start:src_col_end]

    def detect_changes(self, prev_grid, new_grid):
        """Identify meaningful changes between maps"""
        # if self.ignore_unexplored:
        #     mask = (prev_grid != new_grid) & (new_grid != -1)
        # else:
        mask = (prev_grid != new_grid)
        return np.where(mask, new_grid, -1)

    def calculate_bounding_box(self, delta_grid):
        """Compute region of interest with buffer"""
        rows, cols = np.where(delta_grid != -1)
        if rows.size == 0 or cols.size == 0:
            return None

        min_x, max_x = cols.min(), cols.max()
        min_y, max_y = rows.min(), rows.max()

        # Apply buffer with boundary checks
        min_x = max(0, min_x - self.buffer_size)
        max_x = min(delta_grid.shape[1]-1, max_x + self.buffer_size)
        min_y = max(0, min_y - self.buffer_size)
        max_y = min(delta_grid.shape[0]-1, max_y + self.buffer_size)

        return (min_x, min_y, max_x, max_y)

    def crop_delta(self, delta_grid, bbox):
        """Extract relevant region from delta grid"""
        min_x, min_y, max_x, max_y = bbox
        return delta_grid[min_y:max_y+1, min_x:max_x+1]

    # def validate_crop_size(self, cropped_grid):
    #     """Ensure delta size within limits"""
    #     if cropped_grid.shape[0] > self.max_delta_size or \
    #        cropped_grid.shape[1] > self.max_delta_size:
    #         self.get_logger().warn(f"Oversized delta ignored: {cropped_grid.shape}")
    #         return False
    #     return True

    def publish_change(self, delta_grid, map_info, header, bbox):
        """Publish cropped changes"""
        min_x, min_y, _, _ = bbox
        cropped_info = deepcopy(map_info)
        cropped_info.width = delta_grid.shape[1]
        cropped_info.height = delta_grid.shape[0]
        cropped_info.origin.position.x += map_info.resolution * min_x
        cropped_info.origin.position.y += map_info.resolution * min_y

        delta_msg = OccupancyGrid()
        delta_msg.header = header
        delta_msg.info = cropped_info
        delta_msg.data = delta_grid.flatten().astype(np.int8).tolist()

        self.delta_pub.publish(delta_msg)
        self.get_logger().info(f"Published delta: {delta_grid.shape} cells")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapUpdateBroadcast()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()