import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt
import os

class MapExplorer(Node):

    def __init__(self):

        # super() allows the MapExplorer class to inherit all methods and properties from the Node class
        super().__init__('map_explorer')

        # initialise values else error
        self.map_data = None  # Initialize map_data to None
        self.map_height = 0  # Initialize map_height to 0
        self.map_width = 0  # Initialize map_width to 0

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map', 
            self.listener_callback,
            50)
        
        self.bt_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            50
        )

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'pose',
            self.pose_callback,
            20
        )

        self.map_subscription  # prevent unused variable warning
        self.bt_subscription  # prevent unused variable warning
        self.pose_subscription # prevent unused variable warning

        self.pose_publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            50
        )

        self.pose_publisher_

        # creating the explored grid with the same dimensions as the map
        self.explored_grid = np.full((self.map_height, self.map_width), -1)

        self.path_compute_error = False

        self.last_waypoint = []

    def pose_callback(self, msg):
        print("in pose callback")
        # get robot position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z # not sure if z or w
        print(f'x: {self.robot_x}, y: {self.robot_y}, z: {self.robot_z}')
        return
    
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

        # Create a Matplotlib figure and axes
        fig, (ax1, ax2) = plt.subplots(1, 2)

        # Rotate the map 90 degrees counterclockwise and flip left-right to display properly
        rotated_map =  np.fliplr(np.rot90(self.map_2d_array, k=1))
        cax = ax1.imshow(rotated_map, cmap=cmap, norm=norm)

        # Create a colorbar legend
        # cbar = plt.colorbar(cax, cmap=cmap, norm=norm, boundaries=bounds, ticks=[-1, 0, 100])
        # cbar.set_label('Occupancy Values')

        # Set the extent of ax2 to match the extent of ax1 (occupancy grid)
        ax2.set_xlim(ax1.get_xlim())
        ax2.set_ylim(ax1.get_ylim())

        # Set the aspect ratio of both axes to equal
        ax1.set_aspect('equal')
        ax2.set_aspect('equal')

        if self.frontiers:
            # Rotate the frontiers and plot them in black on ax2 in the opposite direction
            frontiers_x, frontiers_y = zip(*self.frontiers)
            rotated_frontiers = [(self.map_height - y, self.map_width - x) for x, y in zip(frontiers_x, frontiers_y)]
            ax2.scatter(*zip(*rotated_frontiers), c='black', marker='o', s=10)

        if self.scores: # make sure self.scores is not empty
            highest_score_frontier = max(self.scores, key=self.scores.get)
            x, y = highest_score_frontier

            # Rotate the highest score point and plot it on ax2 in the opposite direction
            rotated_highest_score = (self.map_height - y, self.map_width - x)
            ax2.plot(*rotated_highest_score, color='pink', marker='x', linestyle='-')

            save_dir = "/home/jasper/turtlebot3_ws/src/map_explorer/map_explorer"
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            save_file = os.path.join(save_dir, "map_plot.png")

            # Save the plot as an image file
            plt.savefig(save_file)

            print(f"Saved figure to {save_file}")
            # Close the Matplotlib figure to release resources
            plt.close()





    # Fix 8 point frontier to 4 point frontier
    def frontier_finder(self):
        # for each -1 (unexplored) cell, see if neighboring 4 cells are 0, if so then mark the unexplored cell as a frontier

        self.frontiers = set()  # set not list because it can avoid duplicate frontiers

        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.map_2d_array[y, x] == -1:
                    # Check the 4 surrounding cells (top, bottom, left, and right)
                    neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
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
    def bt_log_callback(self, msg:BehaviorTreeLog):
        # check navigation status, call send_waypoint if IDLE status
        print("in bt callback")
        number_events = 0
        for event in msg.event_log:
        #     number_events+=1
        #     if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
        #         print("found out that is idling")
        #         self.update_waypoints()
        # print(f'{number_events} events recorded')

            # if in navigation recovery mode, update waypoint
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                print("found out that NavigateRecovery is idling")
                self.path_compute_error = True
                self.update_waypoints()



    def update_waypoints(self):
        print("updating waypoint")
        if not self.scores:
            return  # No scores available, nothing to publish

        highest_score_frontier = max(self.scores, key=self.scores.get)

        # check if could not compute path to pose, then update waypoint to second highest score
        if self.path_compute_error and self.scores: # add condition to check that self.scores is not empty
            print(f"removing path error waypoint: {highest_score_frontier}")
            self.scores.pop(highest_score_frontier)  # Remove the highest score frontier
            if self.scores: # check that self.scores is not empty
                highest_score_frontier = max(self.scores, key=self.scores.get)
                print(f"now going to {highest_score_frontier}")
            else:
                print("done for the day")

        x, y = highest_score_frontier

        map_x, map_y = self.robot_to_map_coordinates(x, y)

        # Create the waypoint message
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = "map"
        print(f'(x,y) pose coordinate: {map_x}, {map_y}\n')
        waypoint_msg.pose.position.x = map_x  # Set x-coordinate
        waypoint_msg.pose.position.y = map_y  # Set y-coordinate

        # do 180 for fun
        # if self.path_arrived:
        #     print("I'm dancing \n \n \n \n \n I'm dancing!!!")
        #     # waypoint_msg.pose.orientation.w = -1.0 * self.robot_z # 180 of current orientation

        # Publish the waypoint
        self.pose_publisher_.publish(waypoint_msg)

        return None



def main(args=None):
    rclpy.init(args=args)
    map_explorer = MapExplorer()
    rclpy.spin(map_explorer)


if __name__ == '__main__':
    main()
