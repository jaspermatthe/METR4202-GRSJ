import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer_node')

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
        # Implement logic to get the robot's current position in the map

    def navigate_to_goal(self, goal_pose):
        # Implement navigation to the goal_pose using move_base or similar
        # Implement obstacle avoidance using the LaserScan data

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
