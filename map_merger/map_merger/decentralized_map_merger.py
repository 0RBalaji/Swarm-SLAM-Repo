import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.ndimage import rotate
import math


def align_maps(node, base_map, new_map, ns):
    """Align new_map to the frame of base_map using TF lookup with namespaces."""
    
    dx = (new_map.info.origin.position.x - base_map.info.origin.position.x)
    dy = (new_map.info.origin.position.y - base_map.info.origin.position.y)

    # Reshape and rotate the map
    new_array = np.array(new_map.data, dtype=np.int8).reshape(
        (new_map.info.height, new_map.info.width)
    )
    
    aligned_map = new_map
    aligned_map.info.origin.position.x += dx
    aligned_map.info.origin.position.y += dy
    aligned_map.data = new_array.flatten().tolist()

    return aligned_map


def merge_maps(map1, map2):
    """Merge two occupancy grid maps safely."""
    merged_map = OccupancyGrid()
    merged_map.header = map1.header
    merged_map.header.frame_id = map1.header.frame_id

    min_x = min(map1.info.origin.position.x, map2.info.origin.position.x)
    min_y = min(map1.info.origin.position.y, map2.info.origin.position.y)
    max_x = max(map1.info.origin.position.x + map1.info.width * map1.info.resolution,
                map2.info.origin.position.x + map2.info.width * map2.info.resolution)
    max_y = max(map1.info.origin.position.y + map1.info.height * map1.info.resolution,
                map2.info.origin.position.y + map2.info.height * map2.info.resolution)

    merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)
    merged_map.info.width = int((max_x - min_x) / merged_map.info.resolution)
    merged_map.info.height = int((max_y - min_y) / merged_map.info.resolution)
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y

    # Initialize merged map data
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

    # Helper function to populate the merged map
    for y in range(map1.info.height):
        for x in range(map1.info.width):
            i = x + y * map1.info.width
            merged_x = int(np.floor((map1.info.origin.position.x + x * map1.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map1.info.origin.position.y + y * map1.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            merged_map.data[merged_i] = map1.data[i]
    for y in range(map2.info.height):
        for x in range(map2.info.width):
            i = x + y * map2.info.width
            merged_x = int(np.floor((map2.info.origin.position.x + x * map2.info.resolution - min_x) / merged_map.info.resolution))
            merged_y = int(np.floor((map2.info.origin.position.y + y * map2.info.resolution - min_y) / merged_map.info.resolution))
            merged_i = merged_x + merged_y * merged_map.info.width
            if merged_map.data[merged_i] == -1:
                merged_map.data[merged_i] = map2.data[i]
    
    return merged_map


class DecentralizedMapMerger(Node):
    def __init__(self):
        super().__init__('map_merger_node')

        self.declare_parameter('namespace', '', ParameterDescriptor(description='Namespace of the robot'))
        self.namespace = self.get_parameter('namespace').value

        if not self.namespace:
            self.get_logger().error("Namespace parameter is required.")
            raise RuntimeError("Namespace parameter missing.")

        self.shared_map = {}  # Store remote maps by namespace
        self.local_map = None
        self.merged_map = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.local_map_sub = self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/map',
            self.local_map_callback,
            self.qos_profile
        )
        
        self.merged_map_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.namespace}/map',
            self.qos_profile
        )
        
        # self.metadata_pub = self.create_publisher(
        #     MapMetaData, 
        #     f'/{self.namespace}/map_metadata', 
        #     self.qos_profile)

        self.create_timer(5.0, self.check_for_new_namespaces)

    def local_map_callback(self, msg):
        self.local_map = msg
        # self.map_merge()

    def check_for_new_namespaces(self):
        """Discover and subscribe to new namespaces."""
        active_topics = self.get_topic_names_and_types()
        remote_topics = [t[0] for t in active_topics if t[0].endswith('/map_changes')]

        # self.get_logger().info(f"remote_topics {remote_topics}")

        for topic in remote_topics:
            ns = topic.split('/')[1]
            # self.get_logger().info(f"ns1 - {ns}")
            if ns and ns != self.namespace and ns not in self.shared_map:
                # self.get_logger().info(f"ns2 - {ns}")
                self.shared_map[ns] = None
                self.create_subscription(OccupancyGrid, topic, lambda msg, ns=ns: self.remote_map_callback(msg, f'/{ns}/map'), 10)
                self.get_logger().info(f"Subscribed to remote topics-> /{ns}/map_chnages")

    def remote_map_callback(self, msg, ns):
        self.shared_map[ns] = msg
        self.map_merge(ns)

    def map_merge(self, nss):
        """Merge local and remote maps."""
        if not self.local_map:
            self.get_logger().warning("Local map not available.")
            return

        merged_map = self.local_map
        for ns, remote_map in self.shared_map.items():
            if remote_map:
                # aligned_map = align_maps(self, self.local_map, remote_map, ns)
                merged_map = merge_maps(merged_map, remote_map)

                self.merged_map = merged_map
                stats_ = self.publish_merged_map(merged_map)
                if stats_:
                    self.get_logger().info(f"Merged map published from {nss} --> {self.namespace}.")
                else:
                    self.get_logger().info(f"ERROR!!! or WARNING!!! or INFO!!! (WHATEVER)!!! --Merged map did not get published from {nss} --> {self.namespace}.")
                
    def publish_merged_map(self, merged_map):
        """Publish the merged map."""
        if merged_map:
            future_offset = rclpy.duration.Duration(seconds=0.1)  # Adjust offset as needed

            # self.merged_map.header.stamp = (self.get_clock().now() + future_offset).to_msg()

            # metadata = MapMetaData()
            
            # metadata.resolution = merged_map.info.resolution
            # metadata.width = merged_map.info.width
            # metadata.height = merged_map.info.height
            # metadata.origin = merged_map.info.origin
            # metadata.header = merged_map.header

            # metadata.map_load_time = self.get_clock().now().to_msg()
            self.merged_map.header.stamp = self.get_clock().now().to_msg()

            self.merged_map_pub.publish(merged_map)
            
            # self.metadata_pub.publish(metadata)
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedMapMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down map merger.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
