import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np

class MapUpdateBroadcast(Node):
    def __init__(self):
        super().__init__('map_update_broadcast')

        # Declare and retrieve namespace parameter
        self.declare_parameter(
            'namespace', '', ParameterDescriptor(description='Namespace of the robot')
        )
        self.namespace = self.get_parameter('namespace').value

        if not self.namespace:
            self.get_logger().error("Namespace parameter is required. Shutting down.")
            rclpy.shutdown()
            return

        # Publisher for broadcasting changes
        self.delta_pub = self.create_publisher(
            OccupancyGrid, f'/{self.namespace}/map_changes', 10
        )

        # Variables to store maps
        self.previous_map = None

        # Subscribe to the map topic
        map_topic = f'/{self.namespace}/map'
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, 10
        )
        self.get_logger().info(f"Subscribed to {map_topic}")

    def map_callback(self, new_map: OccupancyGrid):
        """Callback to process and broadcast changes in the map."""
        try:
            if self.previous_map is None:
                self.previous_map = new_map
                self.get_logger().info("Stored the initial map.")
                return

            self.get_logger().info("Processing new map update.")

            # Align maps for comparison
            aligned_prev, aligned_new, info = self.align_maps(self.previous_map, new_map)

            # Detect changes (any difference)
            delta = (aligned_prev != aligned_new).astype(np.int8) * aligned_new

            if not np.any(delta):  # No changes
                self.get_logger().info("No changes detected.")
                return

            # Compute bounding box
            min_x, min_y, max_x, max_y = self.get_bounding_box(delta)

            # Crop the delta map
            cropped_delta = delta[min_y:max_y + 1, min_x:max_x + 1]
            cropped_info = self.create_cropped_info(info, min_x, min_y, cropped_delta)

            # Publish the changes
            delta_msg = self.create_occupancy_grid(cropped_delta, cropped_info, new_map.header)
            self.delta_pub.publish(delta_msg)
            self.get_logger().info("Published map changes.")

            # Update the previous map
            self.previous_map = new_map

        except Exception as e:
            self.get_logger().error(f"Error processing map update: {e}")

    def align_maps(self, prev_map, new_map):
        """Aligns two maps for comparison."""
        prev_info, new_info = prev_map.info, new_map.info

        # Calculate unified boundaries
        min_x = min(prev_info.origin.position.x, new_info.origin.position.x)
        min_y = min(prev_info.origin.position.y, new_info.origin.position.y)
        max_x = max(prev_info.origin.position.x + prev_info.width * prev_info.resolution,
                    new_info.origin.position.x + new_info.width * new_info.resolution)
        max_y = max(prev_info.origin.position.y + prev_info.height * prev_info.resolution,
                    new_info.origin.position.y + new_info.height * new_info.resolution)

        width = int(np.ceil((max_x - min_x) / prev_info.resolution))
        height = int(np.ceil((max_y - min_y) / prev_info.resolution))

        aligned_prev = np.full((height, width), -1, dtype=np.int8)
        aligned_new = np.full((height, width), -1, dtype=np.int8)

        def copy_to_aligned(map_data, map_info, aligned, min_x, min_y):
            for y in range(map_info.height):
                for x in range(map_info.width):
                    aligned_x = int((map_info.origin.position.x - min_x) / map_info.resolution) + x
                    aligned_y = int((map_info.origin.position.y - min_y) / map_info.resolution) + y
                    aligned[aligned_y, aligned_x] = map_data[y * map_info.width + x]

        copy_to_aligned(prev_map.data, prev_info, aligned_prev, min_x, min_y)
        copy_to_aligned(new_map.data, new_info, aligned_new, min_x, min_y)

        info = prev_info
        info.width = width
        info.height = height
        info.origin.position.x = min_x
        info.origin.position.y = min_y

        return aligned_prev, aligned_new, info

    def get_bounding_box(self, delta):
        """Calculate the bounding box for changes."""
        rows, cols = np.where(delta != 0)
        min_x, max_x = cols.min(), cols.max()
        min_y, max_y = rows.min(), rows.max()
        return min_x, min_y, max_x, max_y

    def create_cropped_info(self, info, min_x, min_y, cropped_grid):
        """Adjust metadata for the cropped map."""
        info.width = cropped_grid.shape[1]
        info.height = cropped_grid.shape[0]
        info.origin.position.x += min_x * info.resolution
        info.origin.position.y += min_y * info.resolution
        return info

    def create_occupancy_grid(self, grid, info, header):
        """Create OccupancyGrid message."""
        msg = OccupancyGrid()
        msg.header = header
        msg.info = info
        msg.data = grid.flatten().tolist()
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = MapUpdateBroadcast()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
