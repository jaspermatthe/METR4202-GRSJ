import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt
import os
import time

class MappingNode(Node):
    def __init__(self):
        super().__init__('map_explorer_2')

        # Initialize variables and data structures
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.map_2d_occupancy_array = None
        self.robot_x = 0
        self.robot_y = 0
        self.robot_z = 0

        # Initialize subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            20)

        self.bt_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_callback,
            20
        )

        self.pose_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.pose_callback,
            10
        )

        # Initialize publishers
        self.goal_publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        # Initialize flags
        self.bt_callback_check = False
        self.pose_callback_check = False
        self.map_callback_check = False
        self.patience = False

        # Create a timer to run the robot_brain at a fixed rate (e.g., every 1 second)
        self.timer_period = 1  # in seconds
        self.timer = self.create_timer(self.timer_period, self.robot_brain)
        self.last_robot_brain_time = time.time()

    # Map callback to process map data
    def map_callback(self, msg):
        self.map_callback_check = True
        self.patience = False

        self.map_data = msg
        self.map_width = self.map_data.info.width
        self.map_height = self.map_data.info.height
        self.map_resolution = self.map_data.info.resolution
        self.map_origin_x = self.map_data.info.origin.position.x
        self.map_origin_y = self.map_data.info.origin.position.y

        self.map_2d_occupancy_array = np.array(self.map_data.data).reshape((self.map_height, self.map_width))

    # Odometry callback to update robot's pose
    def pose_callback(self, msg):
        self.pose_callback_check = True

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z

    # Behavior Tree callback to update robot's behavior
    def bt_callback(self, msg):
        self.bt_callback_check = True
        pass

    # Find unoccupied-unexplored frontiers
    def find_waypoints(self):
        # Your code for finding unoccupied-unexplored frontiers
        pass

    # Score frontiers based on distance to robot
    def score_waypoints(self):
        # Your code for scoring frontiers based on distance to the robot
        pass

    # Helper function to transform grid coordinates to map coordinates
    def grid_to_map_coordinates(self, grid_coordinates_set):
        map_coordinates_set = set()
        for x, y in grid_coordinates_set:
            map_x = x * self.map_resolution + self.map_origin_x
            map_y = y * self.map_resolution + self.map_origin_y
            map_coordinates_set.add((map_x, map_y))
        return map_coordinates_set
    
    # Helper function to transform map coordinates to grid coordinates
    def map_to_grid_coordinates(self, map_coordinates):
        x, y = map_coordinates
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y
    
    # Send the highest-scoring waypoint as a goal
    def send_waypoint(self):
        # Your code to send the highest-scoring waypoint as a goal
        pass

    # Robot brain to control the exploration strategy
    def robot_brain(self):
        current_time = time.time()
        time_diff = current_time - self.last_robot_brain_time

        # If no map callback has happened, wait and send waypoint to origin in the meantime
        if not self.map_callback_check:
            return
        
        # Find waypoints, score them, and send the highest-scoring waypoint as a goal
        self.find_waypoints()
        self.score_waypoints()
        self.send_waypoint()
        self.plot_map()

        # Update the last robot_brain time
        self.last_robot_brain_time = current_time

    # Function to plot the occupancy grid and frontier points
    def plot_map(self):
    # Create a colormap for unexplored (-1, red), unoccupied (0, green), and obstacle (100, blue)
    cmap = plt.cm.colors.ListedColormap(['red', 'green', 'blue'])
    bounds = [-1, 0, 100, 101]  # Define the color boundaries
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    # Create figure with two sublots: one for occupancy grid, second for frontier and goal pose plotting
    fig, (ax1, ax2) = plt.subplots(1, 2)

    # Plot occupancy array on the first figure (ax1)
    self.rotated_map =  np.fliplr(np.rot90(self.map_2d_occupancy_array, k=1))
    ax1.imshow(self.rotated_map, cmap=cmap, norm=norm)

    # Set the extent of ax2 to match the extent of ax1
    ax2.set_xlim(ax1.get_xlim())
    ax2.set_ylim(ax1.get_ylim())

    # Set the aspect ratio of both axes to equal
    ax1.set aspect('equal')
    ax2.set aspect('equal')

    # Scatter plot the unoccupied-unexplored frontiers on the second figure (ax2)
    # Also plot the goal_pose waypoint for the densest frontier
    if self.unoc_unex:
        # Rotate and transform coordinates
        unoc_unex_x, unoc_unex_y = zip(*self.unoc_unex)
        unoc_unex_rotated= [(self.map_height - y, self.map_width - x) for x, y in zip(unoc_unex_x, unoc_unex_y)]
        ax2.scatter(*zip(*unoc_unex_rotated), c='pink', marker='x', s=3)

        # Plot goal_pose waypoint also convert to grid coordinates
        highest_scoring frontier = max(self.dense_frontier_scores, key=self.dense_frontier_scores.get)
        x, y = self.map_to_grid_coordinates(highest_scoring frontier)
        rotated_highest_scoring frontier = [(self.map_height - y, self.map_width - x)]
        ax2.scatter(*zip(*rotated_highest_scoring frontier), color='red', marker='x', linestyle='-')

    # Scatter plot the unoccupied-unexplored-obstacle frontiers on the second figure (ax2)
    if self.unoc_unex_obst:
        # Rotate and transform coordinates
        unoc_unex_obst_x, unoc_unex_obst_y = zip(*self.unoc_unex_obst)
        unoc_unex_obst_rotated= [(self.map_height - y, self.map_width - x) for x, y in zip(unoc_unex_obst_x, unoc_unex_obst_y)]
        ax2.scatter(*zip(*unoc_unex_obst_rotated), c='teal', marker='s', s=5)

        # Plot wall goal pose
        highest_scoring frontier = max(self.wall_frontier_scores, key=self.wall_frontier_scores.get)
        x, y = self.map_to_grid_coordinates(highest_scoring frontier)
        rotated_highest_scoring frontier = [(self.map_height - y, self.map_width - x)]
        ax2.scatter(*zip(*rotated_highest_scoring frontier), color='yellow', marker='o', linestyle='-')

    # Save file
    save_dir = "/home/jasper/turtlebot3_ws/src/map_explorer/map_explorer"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    save_file = os.path.join(save_dir, "map_plot.png")
    plt.savefig(save_file, dpi=200)
    print(f"Saved figure to {save_file}")
    plt.close()

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
