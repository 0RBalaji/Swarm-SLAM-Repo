import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformListener, LookupException
import numpy as np

from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import TransformStamped


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

        # Has both the publisher and subscriber use the same QoS settings. For map topics, TRANSIENT_LOCAL is generally recommended.
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribers and publishers
        self.merged_map_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.namespace}/map',
            qos_profile
            )
        
        self.local_map_sub = self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/map',
            self.local_map_callback,
            qos_profile
        )
        self.get_logger().info(f"Subscribed to local map: /{self.namespace}/map")

        # Store map data and namespace discovery
        self.local_map_data = None
        self.namespaces = set([self.namespace])  # Ensure local namespace is included
        self.remote_subscribers = {}

        # Timer for namespace discovery
        self.discovery_timer = self.create_timer(2.0, self.check_for_new_namespaces)

        self.get_logger().info("Decentralised map merger initialized.")

    def local_map_callback(self, msg):
        """Initialize and update the local map."""
        self.local_map_data = msg
        self.get_logger().info("Local map initialized.")

    def check_for_new_namespaces(self):
        """Discover namespaces and subscribe to their new features."""
        discovered_namespaces = self.find_active_namespaces()
        new_namespaces = discovered_namespaces - self.namespaces

        for ns in new_namespaces:
            if ns != self.namespace:
                self.namespaces.add(ns)
                self.subscribe_to_new_features(ns)

    def find_active_namespaces(self):
        """Find active namespaces dynamically based on topics."""
        active_topics = self.get_topic_names_and_types()
        namespaces = set()
        for topic, _ in active_topics:
            if topic.endswith('/new_features'):
                ns = topic.split('/')[1]
                namespaces.add(ns)
        return namespaces

    def subscribe_to_new_features(self, ns):
        """Subscribe to new features from a remote namespace."""
        topic_name = f'/{ns}/new_features'
        self.get_logger().info(f"Subscribing to new features from namespace: {ns}")
        self.remote_subscribers[ns] = self.create_subscription(
            OccupancyGrid,
            topic_name,
            lambda msg, ns=ns: self.remote_feature_callback(ns, msg),
            10
        )

    def remote_feature_callback(self, ns, msg):
        """Handle new features from a remote namespace."""
        if not self.local_map_data:
            self.get_logger().warning("Local map not initialized. Skipping merge.")
            return

        self.get_logger().info(f"Received new features from namespace: {ns}")

        # # Retrieve transform for positioning remote features
        transform = self.get_tf_for_namespace(ns)
        if transform:
            self.merge_new_features(ns, msg, transform)
        # self.merge_new_features(ns, msg, transform)

    def get_tf_for_namespace(self, ns):
        """Retrieve transform between local and remote map frames."""
        target_frame = f"{self.namespace}/map"  # Local map frame
        source_frame = f"{ns}/map"  # Remote map frame

        try:
            self.get_logger().info(f"Looking up transform from:::: {target_frame} to:::: {source_frame}")
            # transform: TransformStamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

            transform = TransformStamped()
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

            self.get_logger().info(f"Transform lookup successful: {transform}")
            return transform
        except LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return None

    def merge_new_features(self, ns, feature_msg, transform):
        """Merge the new features into the local map."""
        self.get_logger().info(f"Merging new features from namespace: {ns}")

        local_map = np.array(self.local_map_data.data, dtype=np.int8).reshape(
            (self.local_map_data.info.height, self.local_map_data.info.width)
        )
        remote_features = np.array(feature_msg.data, dtype=np.int8).reshape(
            (feature_msg.info.height, feature_msg.info.width)
        )

        # Calculate bounds for the merged map
        min_x = min(self.local_map_data.info.origin.position.x, feature_msg.info.origin.position.x)
        min_y = min(self.local_map_data.info.origin.position.y, feature_msg.info.origin.position.y)
        max_x = max(self.local_map_data.info.origin.position.x + self.local_map_data.info.width * self.local_map_data.info.resolution,
                    feature_msg.info.origin.position.x + feature_msg.info.width * feature_msg.info.resolution)
        max_y = max(self.local_map_data.info.origin.position.y + self.local_map_data.info.height * self.local_map_data.info.resolution,
                    feature_msg.info.origin.position.y + feature_msg.info.height * feature_msg.info.resolution)

        resolution = self.local_map_data.info.resolution
        merged_width = int(np.ceil((max_x - min_x) / resolution))
        merged_height = int(np.ceil((max_y - min_y) / resolution))

        # Initialize the merged map
        merged_map = np.full((merged_height, merged_width), -1, dtype=np.int8)

        # Offset calculations
        x_offset_local = int((self.local_map_data.info.origin.position.x - min_x) / resolution)
        y_offset_local = int((self.local_map_data.info.origin.position.y - min_y) / resolution)
        x_offset_remote = int((feature_msg.info.origin.position.x - min_x) / resolution)
        y_offset_remote = int((feature_msg.info.origin.position.y - min_y) / resolution)

        # Copy local map into merged map
        for y in range(self.local_map_data.info.height):
            for x in range(self.local_map_data.info.width):
                merged_map[y + y_offset_local, x + x_offset_local] = local_map[y, x]

        # Copy remote features into merged map
        for y in range(feature_msg.info.height):
            for x in range(feature_msg.info.width):
                if remote_features[y, x] != -1:  # Only merge valid data
                    merged_map[y + y_offset_remote, x + x_offset_remote] = remote_features[y, x]

        # Flatten the merged map back to a list and update local map data
        self.local_map_data.data = merged_map.flatten().tolist()
        self.local_map_data.info.width = merged_width
        self.local_map_data.info.height = merged_height
        self.local_map_data.info.origin.position.x = min_x
        self.local_map_data.info.origin.position.y = min_y

        # Publish the updated map
        self.merged_map_pub.publish(self.local_map_data)
        self.get_logger().info("Updated local map published.")

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