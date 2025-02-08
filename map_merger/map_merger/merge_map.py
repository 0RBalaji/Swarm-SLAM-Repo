import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

from rcl_interfaces.msg import ParameterDescriptor

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def merge_maps(map1, map2):
    merged_map = OccupancyGrid()
    merged_map.header = map1.header
    merged_map.header.frame_id = 'merge_map'
    min_x = min(map1.info.origin.position.x, map2.info.origin.position.x)
    min_y = min(map1.info.origin.position.y, map2.info.origin.position.y)
    max_x = max(map1.info.origin.position.x + (map1.info.width * map1.info.resolution),
                map2.info.origin.position.x + (map2.info.width * map2.info.resolution))
    max_y = max(map1.info.origin.position.y + (map1.info.height * map1.info.resolution),
                map2.info.origin.position.y + (map2.info.height * map2.info.resolution))
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)
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

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')

        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)

        self.maps = {'/botA':'', '/botB':'', '/botC':'', '/botD':''}
        self.maps = {ns + '/map' for ns in self.maps.keys()}
        self.merged_map = None
        self.map_callbacks()

        self.tf_broadcaster = TransformBroadcaster(self)
    
    def map_callbacks(self):
        for topic_name in self.maps:
            self.get_logger().info(f"Subscribing to {topic_name}")
            self.create_subscription(OccupancyGrid, topic_name, self.map_callback, 10)

    def map_callback(self, msg):
        
        if (self.merged_map == None):
            self.merged_map = msg

        self.merged_map = merge_maps(self.merged_map, msg)

        self.publisher.publish(self.merged_map)
        
        self.get_logger().info("Published!!!")
        
def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
