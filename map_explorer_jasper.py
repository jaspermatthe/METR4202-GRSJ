import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt

class MapExplorer(Node):

    def __init__(self):

        # super() allows the MapExplorer class to inherit all methods and properties from the Node class
        super().__init__('map_explorer')

        # initialise values else error
        self.map_data = None  # Initialize map_data to None
        self.map_height = 0  # Initialize map_height to 0
        self.map_width = 0  # Initialize map_width to 0

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map', 
            self.listener_callback,
            10)
        
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behaviour_tree_log',
            self.bt_log_callback,
            10
        )

        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        # creating the explored grid with the same dimensions as the map
        self.explored_grid = np.full((self.map_height, self.map_width), -1)


    def listener_callback(self, msg):
        # called each time a message is received by the subscription
        
        # reshape map (dimension objects below are created and named as in https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot/blob/main/autonomous_exploration/autonomous_exploration/control.py)
        self.map_data = msg
        self.map_width = self.map_data.info.width
        self.map_height = self.map_data.info.height
        self.map_resolution = self.map_data.info.resolution
        self.map_origin_x = self.map_data.info.origin.position.x
        self.map_origin_y = self.map_data.info.origin.position.y

        self.map_2d_array = np.array(self.map_data.data).reshape((self.map_height, self.map_width))

        # update the explored grid
        self.explored_grid = np.full((self.map_height, self.map_width), -1)

        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.map_2d_array[y, x] == -1 and self.explored_grid[y, x] == -1:
                    # check if the cell is unexplored and hasn't been visited
                    # maybe check also nearby cells?
                    self.explored_grid[y, x] = 0
                    self.frontier_finder()

        # call update frontiers
        self.frontier_finder()

        # score frontiers
        self.score_frontiers()

        # plot occupancy grid
        self.plot_map()

        # update waypoint
        self.update_waypoints()


        return None

    def robot_to_map_coordinates(self, robot_x, robot_y):
        map_x = robot_x * self.map_resolution + self.map_origin_x
        map_y = robot_y * self.map_resolution + self.map_origin_y
        return map_x, map_y
    

    def plot_map(self):
        # Create a colormap for -1 (red), 0 (green), and 100 (blue)
        cmap = plt.cm.colors.ListedColormap(['red', 'green', 'blue'])
        bounds = [-1, 0, 100, 101]  # Define the color boundaries
        norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

        # Create a Matplotlib figure and axis
        fig, (ax1, ax2) = plt.subplots(1, 2)

        # Use imshow to display the 2D array as an image with the specified colormap and norm
        cax = ax1.imshow(self.map_2d_array, cmap=cmap, norm=norm)

        # Create a colorbar legend
        # cbar = plt.colorbar(cax, cmap=cmap, norm=norm, boundaries=bounds, ticks=[-1, 0, 100])
        # cbar.set_label('Occupancy Values')

        # Set the extent of ax2 to match the extent of ax1 (occupancy grid)
        ax2.set_xlim(ax1.get_xlim())
        ax2.set_ylim(ax1.get_ylim())

        # Set the aspect ratio of both axes to equal
        ax1.set_aspect('equal')
        ax2.set_aspect('equal')

        # Plot the frontiers in black as points on ax2
        frontiers_x, frontiers_y = zip(*self.frontiers)
        ax2.scatter(frontiers_x, frontiers_y, c='black', marker='o', s=10)

        highest_score_frontier = max(self.scores, key=self.scores.get)
        x, y = highest_score_frontier

        ax2.plot(x, y, color='pink', marker='x', linestyle='-')


        # Show the plot
        plt.show()


    def frontier_finder(self):
        # for each -1 (unexplored) cell, see if neighbouring 8 cells are 0, if so then mark the unexplored cell as a frontier

        self.frontiers = set() # set not list because can avoid duplicate frontiers

        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.map_2d_array[y, x] == -1:
                    # Check the 8 surrounding cells
                    neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1),
                                 (x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)]
                    for nx, ny in neighbors:
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height and self.map_2d_array[ny, nx] == 0:
                            self.frontiers.add((x, y))
                            break
        return None


    def score_frontiers(self):
        self.scores = {}  # Use a dictionary to store scores for each frontier tuple

        for frontier in self.frontiers:
            x, y = frontier
            score = 0

            # Calculate the score based on proximity to other frontiers
            for other_frontier in self.frontiers:
                if frontier == other_frontier:
                    continue  # Skip comparing a frontier to itself
                other_x, other_y = other_frontier

                # Use Euclidean distance (you can also use Manhattan or other distance metrics)
                distance = ((x - other_x) ** 2 + (y - other_y) ** 2) ** 0.5

                # You may want to use a scaling factor to control the impact of distance on the score
                score += 1 / distance

            self.scores[frontier] = score



    ## function from METR4202 Tutorial 4 document
    def bt_log_callback(self, msg: BehaviorTreeLog):
        # check navigation status, call send_waypoint if IDLE status

        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and \
                    event.current_status == 'IDLE':
                self.update_waypoints()



    def update_waypoints(self):
        if not self.scores:
            return  # No scores available, nothing to publish

        highest_score_frontier = max(self.scores, key=self.scores.get)
        x, y = highest_score_frontier

        map_x, map_y = self.robot_to_map_coordinates(x, y)

        # Create the waypoint message
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = "map"
        print(f'x pose coordinate: {x}\n')
        print(f'y pose coordinate: {y}\n')
        waypoint_msg.pose.position.x = map_x  # Set x-coordinate
        waypoint_msg.pose.position.y = map_y  # Set y-coordinate
        waypoint_msg.pose.orientation.w = 1.0  # Default orientation

        # Publish the waypoint
        self.publisher_.publish(waypoint_msg)

        return None

def main(args=None):
    rclpy.init(args=args)
    map_explorer = MapExplorer()
    rclpy.spin(map_explorer)


if __name__ == '__main__':
    main()

