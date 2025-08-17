#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener
import numpy as np
import copy
from threading import Lock

# def header_stamp_to_float(stamp):
#     return float(stamp.sec) + float(stamp.nanosec) * 1e-9

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

class DecentralizedMapMerger(Node):
    def __init__(self):
        super().__init__('map_merger_node')

        self.declare_parameter('namespace', '', ParameterDescriptor(description='Namespace of the robot'))
        self.namespace = self.get_parameter('namespace').value

        if not self.namespace:
            self.get_logger().error("Namespace parameter is required.")
            raise RuntimeError("Namespace parameter missing.")

        self.qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.shared_map = {}  # Store remote maps by namespace
        self.local_delta = None
        self.base_map = None
        self.merged_map = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.local_delta_sub = self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/map_changes',
            self.local_map_callback,
            # lambda msg: self.delta_callback(msg, namespace),
            self.qos_profile
        )

        # self.local_map_sub = self.create_subscription(
        #     OccupancyGrid,
        #     f'/{self.namespace}/merged_map',
        #     self.local_map_callback,
        #     # lambda msg: self.delta_callback(msg, f'/{self.namespace}/map_changes'),
        #     self.qos_profile
        # )
        
        self.merged_map_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.namespace}/merged_map',
            self.qos_profile
        )

        # self.merged_map = self.local_map
        # self.merged_map_pub.publish(self.merged_map)
        
        # self.metadata_pub = self.create_publisher(
        #     MapMetaData, 
        #     f'/{self.namespace}/map_metadata', 
        #     self.qos_profile)
        self.create_timer(5.0, self.discover_deltas)

    def local_map_callback(self, msg):
        self.local_delta = msg
        self.get_logger().info("Ok, its here")
        self.map_merge()

    def discover_deltas(self):
        """Discover and subscribe to new namespaces."""
        active_topics = self.get_topic_names_and_types()
        remote_topics = [t[0] for t in active_topics if t[0].endswith('/map_changes')]

        for topic in remote_topics:
            ns = topic.split('/')[1]
            if ns and ns != self.namespace and ns not in self.shared_map:
            # if ns and ns not in self.shared_map:
                self.shared_map[ns] = None
                self.create_subscription(OccupancyGrid, topic, lambda msg, ns=ns: self.remote_map_callback(msg, ns), 10)
                self.get_logger().info(f"Subscribed to remote topics-> /{ns}/map_chnages")

    def remote_map_callback(self, msg, ns):
        self.shared_map[ns] = msg
        self.map_merge()

    def map_merge(self):
        """Merge local and remote maps."""
        if self.merged_map is None and self.local_delta is not None:
            self.merged_map = copy.deepcopy(self.local_delta)
            # self.publish_merged_map(self.merged_map)
        
        if self.merged_map is None:
            self.get_logger().warning("No base map available for merging.")
            return
        
        if self.local_delta:
            self.merged_map = self.merge_maps(self.merged_map, self.local_delta)
        
        # Apply remote deltas
        for _, remote_delta in self.shared_map.items():
            if remote_delta:
                self.merged_map = self.merge_maps(self.merged_map, remote_delta)

        # Publish the merged map
        if self.publish_merged_map(self.merged_map):
            self.get_logger().debug(f"Merged map published for namespace {self.namespace}")

    @staticmethod
    def merge_maps(base_map, delta_map):
        """Merge delta_map into base_map, creating expanded map if necessary"""
        if base_map is None:
            return copy.deepcopy(delta_map)
        
        # Calculate expanded boundaries
        min_x = min(base_map.info.origin.position.x, delta_map.info.origin.position.x)
        min_y = min(base_map.info.origin.position.y, delta_map.info.origin.position.y)
        max_x = max(base_map.info.origin.position.x + (base_map.info.width * base_map.info.resolution),
                    delta_map.info.origin.position.x + (delta_map.info.width * delta_map.info.resolution))
        max_y = max(base_map.info.origin.position.y + (base_map.info.height * base_map.info.resolution),
                    delta_map.info.origin.position.y + (delta_map.info.height * delta_map.info.resolution))

        # Create expanded merged map
        merged_map = OccupancyGrid()
        merged_map.header = base_map.header
        merged_map.header.frame_id = base_map.header.frame_id
        # merged_map.header.frame_id = self.namespace + '/merge_map'

        merged_map.info.resolution = min(base_map.info.resolution, delta_map.info.resolution)
        merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
        merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

        for y in range(base_map.info.height):
            for x in range(base_map.info.width):
                i = x + y * base_map.info.width
                merged_x = int(np.floor((base_map.info.origin.position.x + x * base_map.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((base_map.info.origin.position.y + y * base_map.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                merged_map.data[merged_i] = base_map.data[i]
        for y in range(delta_map.info.height):
            for x in range(delta_map.info.width):
                i = x + y * delta_map.info.width
                merged_x = int(np.floor((delta_map.info.origin.position.x + x * delta_map.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((delta_map.info.origin.position.y + y * delta_map.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width
                if merged_map.data[merged_i] == -1:
                    merged_map.data[merged_i] = delta_map.data[i]

        return merged_map

    def publish_merged_map(self, merged_map):
        """Publish the merged map."""

        if merged_map is None:
            return False
        
        # msg = OccupancyGrid()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = merged_map.get('frame_id')

        # info = msg.info
        # info.resolution = float(merged_map['resolution'])
        # info.width = int(merged_map.info.width)
        # info.height = int(merged_map.info.height)
        # info.origin.position.x = float(merged_map['origin_x'])
        # info.origin.position.y = float(merged_map['origin_y'])
        # info.origin.position.z = 0.0

        # flat = self.merged_data.flatten().astype(np.int8).tolist()
        # msg.data = flat
        # self.merged_map_pub.publish(msg)
        # self.get_logger().debug("Published merged_map")

        # metadata.map_load_time = self.get_clock().now().to_msg()
        merged_map.header.stamp = self.get_clock().now().to_msg()

        self.merged_map_pub.publish(merged_map)

        self.get_logger().info("Published See Pa")
        
        # self.metadata_pub.publish(metadata)
        return True

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