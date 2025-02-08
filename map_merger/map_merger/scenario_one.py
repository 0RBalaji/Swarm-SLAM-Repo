import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalNavigation(Node):
    def __init__(self):
        super().__init__('goal_navigation')
        self._client = ActionClient(self, NavigateToPose, '/botA/navigate_to_pose')

    def send_navigation_goals(self):
        # Waypoints to be followed
        goals = [
            {'x': -8.0, 'y': 1.5},  # First goal
            {'x': -8.0, 'y': 1.5},
            {'x': 3.0, 'y': 2.5},   # Second goal
        ]

        # Wait for the action server to be ready
        while not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Waiting for action server...')

        for goal in goals:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.create_pose(goal)

            self.get_logger().info(f'Sending goal: {goal}')
            send_goal_future = self._client.send_goal_async(goal_msg)

            # Wait for user input before sending the next goal
            self.wait_for_user_input()

    def create_pose(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Frame of reference for the map
        pose.pose.position.x = goal['x']
        pose.pose.position.y = goal['y']
        pose.pose.orientation.w = 1.0  # No rotation needed
        return pose

    def wait_for_user_input(self):
        # Wait until user enters "y" in the terminal
        while True:
            user_input = input("Enter 'y' to send the next goal: ")
            if user_input.lower() == 'y':
                break
            else:
                self.get_logger().info("Invalid input. Please enter 'y' to continue.")


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigation()
    node.send_navigation_goals()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
