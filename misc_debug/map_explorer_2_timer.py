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

        # http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html
        # Occupancy probabilities are in the range [0,100]. Unknown is -1.
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            20)

        # https://index.ros.org/p/nav2_msgs/#humble-assets
        self.bt_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_callback,
            20
        )

        # http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
        self.pose_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.pose_callback,
            10
        )

        # Prevent unused variable warnings
        self.map_subscription  
        self.bt_subscription
        self.pose_subscription 

        # http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html
        self.goal_publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        self.bt_callback_check = False
        self.pose_callback_check = False
        self.map_callback_check = False
        self.patience = False


        # Create a timer to run the robot_brain at a fixed rate (e.g., every 1 second)
        self.timer_period = 1  # in seconds
        self.timer = self.create_timer(self.timer_period, self.robot_brain)
        self.last_robot_brain_time = time.time()



    def map_callback(self, msg):
        # Get map info: width, height, resolution, and origin
        # Reshape, re-orientate map, and save map as a 2D occupancy array
        print("in map_callback function")
        self.map_callback_check = True
        self.patience = False

        self.map_data = msg
        self.map_width = self.map_data.info.width
        self.map_height = self.map_data.info.height
        self.map_resolution = self.map_data.info.resolution
        self.map_origin_x = self.map_data.info.origin.position.x
        self.map_origin_y = self.map_data.info.origin.position.y

        self.map_2d_occupancy_array = np.array(self.map_data.data).reshape((self.map_height, self.map_width))

        pass

    def pose_callback(self, msg):
        # Using odometry subscription to get pose because of conflict with PoseStamped publisher
        print("in pose_callback function")
        self.pose_callback_check = True

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z # not sure if z or w
        print(f'pose is x: {self.robot_x}, y: {self.robot_y}, z: {self.robot_z}')
        pass

    def bt_callback(self, msg):
        # Check the behavior tree and update robot's behavior accordinly
        print("in bt_callback function")
        self.bt_callback_check = True

        # for event in msg.event_log:
            # if going to waypoint, do not re-send waypoints until reached or idle
            # if event.node_name == 'FollowPath' and event.current_status == 'RUNNING':
            #     print("going to waypoint, please be patient...")
            #     self.patience = True
            #     self.robot_brain()


            # if reached waypoint, re-find, score, and send waypoint
            # if event.node_name == 'FollowPath' and event.current_status == 'SUCCESS' or event.current_status == 'IDLE':
            #     print("reached waypoint, re-computing path...")
            #     self.patience = False
            #     self.robot_brain()

            # if unable to reach waypoint, re-find, score, and send waypoint
            # if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
            #     print("unable to reach waypoint, re-computing path...")
            #     self.patience = False
            #     self.robot_brain()


        pass

    def find_waypoints(self):
        # Use occupancy array to find waypoints
        print("in find_waypoints function")

        # Find boundary where unnocupied (0), unexplored (-1), and obstacles (100) meet
        self.unoc_unex_obst = set()
        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.map_2d_occupancy_array[y, x] == 0:
                    # check the 4 surrounding cells (top, bottom, left, and right)
                    neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1), (x+1, y+1), (x-1, y-1), (x-1, y+1), (x+1, y-1)]
                    is_unexplored_found = False
                    is_obstacle_found = False

                    for nx, ny in neighbors:
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                            if self.map_2d_occupancy_array[ny, nx] == -1:
                                is_unexplored_found = True
                            elif self.map_2d_occupancy_array[ny, nx] == 100:
                                is_obstacle_found = True

                    if is_unexplored_found and is_obstacle_found:
                        self.unoc_unex_obst.add((x, y))

        # Find boundary between unoccupied (0), unexplored (-1)
        self.unoc_unex = set()
        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.map_2d_occupancy_array[y, x] == 0:
                    # check the 4 surrounding cells (top, bottom, left, and right)
                    neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
                    for nx, ny in neighbors:
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height and self.map_2d_occupancy_array[ny, nx] == -1:
                            self.unoc_unex.add((x, y))
                            break

        pass

    def score_waypoints(self):
        print("in score_waypoints function")

        # Transform frontiers from grid to map coordinates
        self.map_unoc_unex_obst = self.grid_to_map_coordinates(self.unoc_unex_obst)
        self.map_unoc_unex = self.grid_to_map_coordinates(self.unoc_unex)

        # 1. Closest unoccupied-unexplored-obstacle pixels
        # This allows robot to follow walls
        self.wall_frontier_scores = {}
        for frontier in self.map_unoc_unex_obst:
            x, y = frontier
            score = 0

            # Compute frontier distance from current pose
            euclidean_distance = ((x - self.robot_x)**2 + (y - self.robot_y)**2)**0.5

            # Assign highest score to frontier closest to current pose
            # Ensure that euclidean_distance is not zero before division
            if euclidean_distance > 0:
                score = 1 / euclidean_distance

            self.wall_frontier_scores[frontier] = score

        # 2. Densest unoccupied-unexplored pixels and distance to the latter
        self.dense_frontier_scores = {}  # Use a dictionary to store scores for each frontier tuple
        for frontier in self.map_unoc_unex:
            x, y = frontier
            score = 0

            density_weight = 0.5
            distance_weight = 1 - density_weight

            # Calculate the score based on proximity to other frontiers
            for other_frontier in self.map_unoc_unex:
                if frontier == other_frontier:
                    continue  # Skip comparing a frontier to itself
                other_x, other_y = other_frontier

                euclidean_distance = ((x - other_x) ** 2 + (y - other_y) ** 2) ** 0.5

                # You may want to use a scaling factor to control the impact of distance on the score
                # Ensure that euclidean_distance is not zero before division
                if euclidean_distance > 0:
                    score += density_weight* 1 / euclidean_distance

            self.wall_frontier_scores[frontier] = score

            # Compute frontier distance from current pose
            euclidean_distance = ((x - self.robot_x)**2 + (y - self.robot_y)**2)**0.5

            # Assign highest score to frontier closest to current pose
            score += distance_weight * (1 / euclidean_distance)
            

            self.dense_frontier_scores[frontier] = score

        pass

    def grid_to_map_coordinates(self, grid_coordinates_set):
        map_coordinates_set = set()
        for x, y in grid_coordinates_set:
            map_x = x * self.map_resolution + self.map_origin_x
            map_y = y * self.map_resolution + self.map_origin_y
            map_coordinates_set.add((map_x, map_y))
        return map_coordinates_set
    
    def map_to_grid_coordinates(self, map_coordinates):
        x, y = map_coordinates
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y
    
    def send_waypoint(self):
        print("in send_waypoint functions")

        # strategy = "dense"
        strategy = "walls"
        # Create the waypoint message
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = "map"

        # If no waypoints, go to center of map
        if not self.map_callback_check:
            print("sending waypoint to origin")
            waypoint_msg.pose.position.x = 0.0
            waypoint_msg.pose.position.y = 0.0

        elif self.dense_frontier_scores:

            if strategy == "dense":
                print("computing highest density frontier...")
                highest_scoring_frontier = max(self.dense_frontier_scores, key=self.dense_frontier_scores.get)
                x, y = highest_scoring_frontier
                waypoint_msg.pose.position.x = x  
                waypoint_msg.pose.position.y = y

            elif strategy == "walls":
                print("computing closest wall frontier...")
                highest_scoring_frontier = max(self.wall_frontier_scores, key=self.wall_frontier_scores.get)
                x, y = highest_scoring_frontier
                waypoint_msg.pose.position.x = x  
                waypoint_msg.pose.position.y = y

        else:
            print("WELP, we're done")

        print(f"my patience is {self.patience}")

        if self.patience == False:
            print(f'going to (x: {waypoint_msg.pose.position.x}, y: {waypoint_msg.pose.position.y})')
            self.goal_publisher_.publish(waypoint_msg)

        pass




    def robot_brain(self):
        print("in robot_brain function")
        current_time = time.time()
        time_diff = current_time - self.last_robot_brain_time
        print(f"                                              Time since last robot_brain call: {time_diff} seconds")

        # if no map_callback has happened, wait and send waypoint to origin in the meantime
        if not self.map_callback_check:
            print("no map callback yet")
            return
        
        self.find_waypoints()
        self.score_waypoints()
        self.send_waypoint()
        # self.patience = True
        self.plot_map()

        # Update the last robot_brain time
        self.last_robot_brain_time = current_time

    def plot_map(self):
        # Create a colormap for unexplored (-1, red), unoccupied (0, green), and obstacle (100, blue)
        cmap = plt.cm.colors.ListedColormap(['red', 'green', 'blue'])
        bounds = [-1, 0, 100, 101]  # Define the color boundaries
        norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

        # Create figure with two sublots: one for occupancy grid, second for frontier and goal pose plotting
        fig, (ax1, ax2) = plt.subplots(1, 2)

        # Plot occupancy array on first figure (ax1)
        self.rotated_map =  np.fliplr(np.rot90(self.map_2d_occupancy_array, k=1))
        ax1.imshow(self.rotated_map, cmap=cmap, norm=norm)

        # Set the extent of ax2 to match the extent of ax1
        ax2.set_xlim(ax1.get_xlim())
        ax2.set_ylim(ax1.get_ylim())

        # Set the aspect ratio of both axes to equal
        ax1.set_aspect('equal')
        ax2.set_aspect('equal')

        # Scatter plot the unoccupied-unexplored frontiers on the second figure (ax2)
        # Also plot the goal_pose way_point for densest frontier
        if self.unoc_unex:
            # Rotate, and transform coordinates
            unoc_unex_x, unoc_unex_y = zip(*self.unoc_unex)
            unoc_unex_rotated= [(self.map_height - y, self.map_width - x) for x, y in zip(unoc_unex_x, unoc_unex_y)]
            ax2.scatter(*zip(*unoc_unex_rotated), c='pink', marker='x', s=3)

            # Plot goal_pose waypoint also convert to grid coordinates
            highest_scoring_frontier = max(self.dense_frontier_scores, key=self.dense_frontier_scores.get)
            x, y = self.map_to_grid_coordinates(highest_scoring_frontier)
            rotated_highest_scoring_frontier = [(self.map_height - y, self.map_width - x)]
            ax2.scatter(*zip(*rotated_highest_scoring_frontier), color='red', marker='x', linestyle='-')


        # Scatter plot the unoccupied-unexplored-obstacle frontiers on the second figure (ax2)
        if self.unoc_unex_obst:
            # Rotate, and transform coordinates
            unoc_unex_obst_x, unoc_unex_obst_y = zip(*self.unoc_unex_obst)
            unoc_unex_obst_rotated= [(self.map_height - y, self.map_width - x) for x, y in zip(unoc_unex_obst_x, unoc_unex_obst_y)]
            ax2.scatter(*zip(*unoc_unex_obst_rotated), c='teal', marker='s', s=5)


            # Plot wall goal pose
            highest_scoring_frontier = max(self.wall_frontier_scores, key=self.wall_frontier_scores.get)
            x, y = self.map_to_grid_coordinates(highest_scoring_frontier)
            rotated_highest_scoring_frontier = [(self.map_height - y, self.map_width - x)]
            ax2.scatter(*zip(*rotated_highest_scoring_frontier), color='yellow', marker='o', linestyle='-')

            pass
        

        # Save file
        save_dir = "/home/jasper/turtlebot3_ws/src/map_explorer/map_explorer"
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        save_file = os.path.join(save_dir, "map_plot.png")
        plt.savefig(save_file, dpi=200)
        print(f"Saved figure to {save_file}")
        plt.close()

        pass


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
