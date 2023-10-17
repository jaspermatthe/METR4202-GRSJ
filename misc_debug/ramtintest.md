import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from nav2_msgs.msg import NavigateToPose

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.waypoint_publisher = self.create_publisher(
            NavigateToPose, 'navigate_to_pose', 10)

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.occupancy_grid = None
        self.frontier = None
        self.explored = set()
        self.unexplored = set()

    def listener_callback(self, msg):
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.update_frontiers(msg.info.origin.position)

    def update_frontiers(self, map_origin):
        if self.occupancy_grid is None:
            return

        current_position = self.get_robot_position(map_origin)
        self.explored.add(tuple(current_position))

        # Update unexplored frontiers based on the current position
        for x in range(-1, 2):
            for y in range(-1, 2):
                if x == 0 and y == 0:
                    continue

                neighbor = (current_position[0] + x, current_position[1] + y)

                if tuple(neighbor) not in self.explored and self.is_valid(neighbor):
                    self.unexplored.add(tuple(neighbor))

        # Find the furthest frontier
        if self.unexplored:
            self.frontier = max(self.unexplored, key=lambda pos: np.linalg.norm(np.array(pos) - np.array(current_position)))

            # Navigate to the furthest frontier point
            self.navigate_to_goal(self.frontier)

            # Mark the cell as explored
            self.explored.add(self.frontier)

            # Remove the explored frontier from the unexplored set
            self.unexplored.discard(self.frontier)

    def is_valid(self, position):
        if (
            0 <= position[0] < self.occupancy_grid.shape[0] and
            0 <= position[1] < self.occupancy_grid.shape[1] and
            self.occupancy_grid[position[0], position[1]] == -1
        ):
            return True
        return False

    def get_robot_position(self, map_origin):
        try:
            # Get the transform from the robot's frame to the map frame
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame (map frame)
                'base_link',  # Source frame (robot's base frame)
                rclpy.time.Time(0),
                rclpy.duration.Duration(1.0)
            )

            # Create a PoseStamped message representing the robot's position
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = 'base_link'
            robot_pose.pose.position.x = 0.0  # Initialize to zero
            robot_pose.pose.position.y = 0.0
            robot_pose.pose.orientation.w = 1.0

            # Transform the robot's pose to the map frame
            transformed_pose = do_transform_pose(robot_pose, transform)

            # Calculate the robot's position in the map frame
            robot_x = transformed_pose.pose.position.x + map_origin.x
            robot_y = transformed_pose.pose.position.y + map_origin.y

            return (robot_x, robot_y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("Error getting robot position.")
            return None

    def navigate_to_waypoint(self, waypoint):
        if waypoint is not None:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = waypoint[0]
            goal.pose.position.y = waypoint[1]
            goal.pose.orientation.w = 1.0

            navigate_msg = NavigateToPose()
            navigate_msg.target_pose = goal

            self.waypoint_publisher.publish(navigate_msg)

def main(args=None):
    rclpy.init(args=args)
    explorer = ExplorerNode()
    
    while rclpy.ok():
        rclpy.spin_once(explorer)
        if explorer.occupancy_grid is not None and -1 not in explorer.occupancy_grid:
            print("Exploration complete! All cells are explored.")
            break

    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
