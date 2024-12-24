import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from rcl_interfaces.msg import ParameterDescriptor

class MapUpdateBroadcast(Node):
    def __init__(self):
        super().__init__('map_update_broadcast')

        # Declare and retrieve namespace parameter
        self.declare_parameter('namespace', '', ParameterDescriptor(description='Namespace of the robot'))
        self.namespace = self.get_parameter('namespace').value

        if not self.namespace:
            self.get_logger().error("Namespace parameter is required.")
            raise RuntimeError("Namespace parameter missing.")

        # Subscribe to the SLAM Toolbox update topic
        self.update_sub = self.create_subscription(
            OccupancyGridUpdate,
            f'/{self.namespace}/slam_toolbox/update',
            self.update_callback,
            10
        )

        # Publisher for processed map updates
        self.update_pub = self.create_publisher(
            OccupancyGridUpdate,
            f'/{self.namespace}/updates_map',
            10
        )

        # Subscribe to the map topic
        self.get_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.get_logger().info(f"Subscribed to: /{self.namespace}/slam_toolbox/update")
        self.get_logger().info(f"Publishing processed updates to: /{self.namespace}/updates_map")

    def update_callback(self, msg):
        try:
            # Log the received update details
            self.get_logger().info(
                f"Received update: x={msg.x}, y={msg.y}, width={msg.width}, height={msg.height}, data size={len(msg.data)}"
            )

            # Directly forward the update message (or apply any custom processing here)
            self.update_pub.publish(msg)
            self.get_logger().info(f"Published map update: x={msg.x}, y={msg.y}, width={msg.width}, height={msg.height}")

        except Exception as e:
            self.get_logger().error(f"Error in update_callback: {e}")


def main(args=None):
    """Initialize the ROS 2 node."""
    rclpy.init(args=args)
    try:
        node = MapUpdateBroadcast()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down map update broadcast.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
