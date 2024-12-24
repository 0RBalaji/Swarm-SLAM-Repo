import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import Buffer, TransformListener

class DecentralizedMapMerger(Node):
    def __init__(self):
        super().__init__('map_merger_node')

        # Declare and retrieve namespace parameter
        self.declare_parameter('namespace', '', ParameterDescriptor(description='Namespace of the robot'))
        self.namespace = self.get_parameter('namespace').value

        if not self.namespace:
            self.get_logger().error("Namespace parameter is required.")
            raise RuntimeError("Namespace parameter missing.")

        # Initialize variables
        self.local_map = None
        self.subscribed_namespaces = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the local map topic
        self.local_map_sub = self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/map'.strip('/'),
            self.local_map_callback,
            10
        )

        # Timer to periodically discover and subscribe to new namespaces
        self.create_timer(5.0, self.check_for_new_namespaces)

    def local_map_callback(self, msg):
        """Callback to store local map updates."""
        self.local_map = msg

    def check_for_new_namespaces(self):
        """Discover namespaces and subscribe to their map topics."""
        node_info = self.get_node_names_and_namespaces()
        discovered_namespaces = {ns[1][1:] for ns in node_info if ns[1].startswith('/')}

        for ns in discovered_namespaces:
            # Only subscribe to `/map`, skip costmaps and already subscribed namespaces
            if ns not in self.subscribed_namespaces and "costmap" not in ns:
                topic_name = f'/{ns}/map'.strip('/')
                self.get_logger().info(f"Discovered new namespace: {ns}. Subscribing to {topic_name}.")
                self.subscribed_namespaces.add(ns)

                self.create_subscription(
                    OccupancyGrid,
                    topic_name,
                    self.remote_map_callback,
                    10
                )

                # Try to find the map TF of the new namespace
                self.get_tf_for_namespace(ns)

    def remote_map_callback(self, msg):
        """Callback for remote maps."""
        self.get_logger().info(f"Received map from namespace: {msg.header.frame_id}")

    def get_tf_for_namespace(self, ns):
        """Attempt to retrieve the map TF for the given namespace."""
        map_frame = f'/{ns}/map'.strip('/')

        try:
            transform = self.tf_buffer.lookup_transform('map', map_frame, rclpy.time.Time())
            self.get_logger().info(f"Found TF for {ns}: {transform.transform}")
        except Exception as e:
            self.get_logger().warn(f"Could not find TF for namespace {ns}: {e}")

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


