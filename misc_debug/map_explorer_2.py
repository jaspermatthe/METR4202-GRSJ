import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt
import os

class MappingNode(Node):
    def __init__(self):
        super().__init__('map_explorer_2')

        # http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html
        # Occupanc probabilities are in the range [0,100]. Unknown is -1.
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map', 
            self.map_callback,
            10)

        # https://index.ros.org/p/nav2_msgs/#humble-assets
        self.bt_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_callback,
            10
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

        self.map_info = False
        self.bt_callback_check = False
        self.pose_callback_check = False
        self.map_callback_check = False



    def map_callback(self, msg):
        # Get map info: width, height, resolution, and origin
        # Reshape, re-orientate map, and save map as a 2D occupancy array
        print("in map_callback function")
        self.map_callback_check = True


        self.map_data = msg
        if not self.map_info:
            self.map_width = self.map_data.info.width
            self.map_height = self.map_data.info.height
            self.map_resolution = self.map_data.info.resolution
            self.map_origin_x = self.map_data.info.origin.position.x
            self.map_origin_y = self.map_data.info.origin.position.y
            self.map_info = True

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

        pass

    def find_waypoints(self):
        # Use occupancy array to find waypoints

        # Find boundary where unnocupied (0), unexplored (-1), and obstacles (100) meet
        self.unoc_unex_obst = set()
        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.map_2d_occupancy_array[y, x] == 0:
                    # check the 4 surrounding cells (top, bottom, left, and right)
                    neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
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
        # Transform frontiers to map coordinates
        print(self.unoc_unex_obst)
        self.map_unoc_unex_obst = self.grid_to_map_coordinates(self.unoc_unex_obst)
        print(self.map_unoc_unex_obst)

        # Closest unoccupied-unexplored-obstacle pixels
        # this allows robot to map out along the walls

        # Densest unoccupied-unexplored pixels


        pass

    def grid_to_map_coordinates(self, coordinates_set):
        map_coordinates_set = set()
        for x, y in coordinates_set:
            map_x = x * self.map_resolution + self.map_origin_x
            map_y = y * self.map_resolution + self.map_origin_y
            map_coordinates_set.add((map_x, map_y))
        return map_coordinates_set
    
    def send_waypoint(self):
        print("in send_waypoint functions")

        # Create the waypoint message
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = "map"
        waypoint_msg.pose.position.x = 1.5  # Set x-coordinate
        waypoint_msg.pose.position.y = 1.5  # Set y-coordinate

        self.goal_publisher_.publish(waypoint_msg)

        pass

    def robot_brain(self):
        print("in robot_brain function")

        # if no map_callback has happened, wait
        if not self.map_callback_check:
            return
        
        self.find_waypoints()
        self.score_waypoints()
        self.send_waypoint()
        self.plot_map()

    def plot_map(self):
        # Create a colormap for unexplored (-1, red), unoccupied (0, green), and obstacle (100, blue)
        cmap = plt.cm.colors.ListedColormap(['red', 'green', 'blue'])
        bounds = [-1, 0, 100, 101]  # Define the color boundaries
        norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

        # Create figure with two sublots: one for occupancy grid, second for frontier and goal pose plotting
        fig, (ax1, ax2) = plt.subplots(1, 2)

        # Convert coordinate systems


        # Plot
        self.rotated_map =  np.fliplr(np.rot90(self.map_2d_occupancy_array, k=1))
        ax1.imshow(self.rotated_map, cmap=cmap, norm=norm)
        # Set the extent of ax2 to match the extent of ax1 (occupancy grid)
        ax2.set_xlim(ax1.get_xlim())
        ax2.set_ylim(ax1.get_ylim())
        # Set the aspect ratio of both axes to equal
        ax1.set_aspect('equal')
        ax2.set_aspect('equal')

        if self.unoc_unex:
            # Rotate, and transform coordinates
            unoc_unex_x, unoc_unex_y = zip(*self.unoc_unex)
            unoc_unex_rotated= [(self.map_height - y, self.map_width - x) for x, y in zip(unoc_unex_x, unoc_unex_y)]
            ax2.scatter(*zip(*unoc_unex_rotated), c='pink', marker='x', s=5)

        if self.unoc_unex_obst:
            # Rotate, and transform coordinates
            unoc_unex_obst_x, unoc_unex_obst_y = zip(*self.unoc_unex_obst)
            unoc_unex_obst_rotated= [(self.map_height - y, self.map_width - x) for x, y in zip(unoc_unex_obst_x, unoc_unex_obst_y)]
            ax2.scatter(*zip(*unoc_unex_obst_rotated), c='teal', marker='s', s=5)



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
    
    while rclpy.ok():
        rclpy.spin_once(node)
        node.robot_brain() # must use as fallback

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()