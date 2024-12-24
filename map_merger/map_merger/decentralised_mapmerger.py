import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformListener, LookupException
import numpy as np


class DecentralisedMapMerger(Node):
    def __init__(self):
        super().__init__('map_merging_node')

        # Declare and retrieve namespace parameter
        self.declare_parameter('namespace', '', ParameterDescriptor(description='Namespace of the robot'))
        self.namespace = self.get_parameter('namespace').value

        if not self.namespace:
            self.get_logger().error("Namespace parameter is required.")
            raise RuntimeError("Namespace parameter missing.")

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers and publishers
        self.local_map_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.namespace}/merged_map',
            10
        )
        self.local_map_sub = self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/map',
            self.local_map_callback,
            10
        )
        self.get_logger().info(f"Subscribed to local map: /{self.namespace}/map")

        # Store map data and namespace discovery
        self.local_map_data = None
        self.namespaces = set([self.namespace])  # Ensure local namespace is included
        self.remote_subscribers = {}

        # Timer for namespace discovery (only for remote namespaces)
        self.discovery_timer = self.create_timer(2.0, self.check_for_new_namespaces)

        self.get_logger().info("Decentralised map merger initialized.")

    def local_map_callback(self, msg):
        """Initialize and update the local map."""
        self.local_map_data = msg
        self.get_logger().info("Local map initialized.")

    def check_for_new_namespaces(self):
        """Discover namespaces and subscribe to their map updates."""
        discovered_namespaces = self.find_active_namespaces()
        new_namespaces = discovered_namespaces - self.namespaces

        for ns in new_namespaces:
            if ns != self.namespace:
                self.namespaces.add(ns)
                self.subscribe_to_remote_updates_map(ns)

    def find_active_namespaces(self):
        """Find active namespaces dynamically based on topics."""
        active_topics = self.get_topic_names_and_types()
        namespaces = set()
        for topic, _ in active_topics:
            if topic.endswith('/updates_map'):
                ns = topic.split('/')[1]
                namespaces.add(ns)
        return namespaces

    def subscribe_to_remote_updates_map(self, ns):
        """Subscribe to remote map updates from a new namespace."""
        topic_name = f'/{ns}/updates_map'
        self.get_logger().info(f"Subscribing to remote map updates from namespace: {ns}")
        self.remote_subscribers[ns] = self.create_subscription(
            OccupancyGridUpdate,
            topic_name,
            lambda msg, ns=ns: self.remote_map_callback(ns, msg),
            10
        )

    def remote_map_callback(self, ns, msg):
        """Handle remote map updates."""
        if not self.local_map_data:
            self.get_logger().warning("Local map not initialized. Skipping merge.")
            return

        self.get_logger().info(f"Received map update from namespace: {ns}")

        # Now we retrieve the transform information to properly position the received map
        transform = self.get_tf_for_namespace(ns)
        if transform:
            self.map_merge(ns, msg, transform)

    def get_tf_for_namespace(self, ns):
        # Construct the frame IDs
        target_frame = f"{self.namespace}/map"  # Local map frame of the receiver
        source_frame = f"{ns}/map"  # Map frame of the sending robot

        # Perform the transform lookup from the sending robot's map to the receiving robot's map
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return None

    def map_merge(self, ns, update_msg, transform):
        """Merge the received map update with the local map."""
        self.get_logger().info("Performing Meriging")
        self.get_logger().info(f"Merging map from namespace: {ns}")

        local_map = np.array(self.local_map_data.data, dtype=np.int8).reshape(
            (self.local_map_data.info.height, self.local_map_data.info.width)
        )
        remote_map = np.array(update_msg.data, dtype=np.int8).reshape(
            (update_msg.height, update_msg.width)
        )

        # Use the transform to calculate the offset (position of the remote map in the local map's coordinate frame)
        x_offset = int(transform.transform.translation.x / self.local_map_data.info.resolution)
        y_offset = int(transform.transform.translation.y / self.local_map_data.info.resolution)

        for y in range(update_msg.height):
            for x in range(update_msg.width):
                local_x = x + x_offset + update_msg.x
                local_y = y + y_offset + update_msg.y
                if 0 <= local_x < local_map.shape[1] and 0 <= local_y < local_map.shape[0]:
                    if remote_map[y, x] != -1:  # Only merge valid data
                        local_map[local_y, local_x] = remote_map[y, x]

        self.local_map_data.data = local_map.flatten().tolist()
        self.local_map_pub.publish(self.local_map_data)
        self.get_logger().info(f"Merged map from {ns} published.")
        self.get_logger().info("End of Meriging")


def main(args=None):
    """Initialize the node and start spinning."""
    rclpy.init(args=args)
    try:
        node = DecentralisedMapMerger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down map merger.")
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
