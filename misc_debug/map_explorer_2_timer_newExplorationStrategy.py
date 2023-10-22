import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import os
import time

class MappingNode(Node):
    def __init__(self):
        super().__init__('map_explorer_2')

        # Subscribe to topics and create publishers
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            20)

        self.pose_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.pose_callback,
            10
        )

        self.goal_publisher_ = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.map_2d_occupancy_array = None

        self.robot_x = 0
        self.robot_y = 0

        self.map_callback_check = False

        # Initialize waypoint strategy
        self.initial_exploration = True
        self.waypoint_strategy = "furthest"

        # Create a timer to run the robot_brain at a fixed rate (e.g., every 1 second)
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.robot_brain)
        self.last_robot_brain_time = time.time()

    def map_callback(self, msg):
        self.map_callback_check = True
        self.map_data = msg
        self.map_width = self.map_data.info.width
        self.map_height = self.map_data.info.height
        self.map_resolution = self.map_data.info.resolution
        self.map_origin_x = self.map_data.info.origin.position.x
        self.map_origin_y = self.map_data.info.origin.position.y
        self.map_2d_occupancy_array = np.array(self.map_data.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def score_waypoints_initial(self):
        # Calculate the distance to the farthest frontier
        farthest_frontier = max(self.map_unoc_unex_obst, key=lambda f: self.euclidean_distance(self.robot_x, self.robot_y, *f))
        distance_to_farthest = self.euclidean_distance(self.robot_x, self.robot_y, *farthest_frontier)

        self.wall_frontier_scores = {}
        for frontier in self.map_unoc_unex_obst:
            x, y = frontier
            score = 0

            # Adjust score higher for distance to farthest frontier
            distance_weight = 1.0  # Modify this weight to balance density and distance
            distance_score = 1 / self.euclidean_distance(self.robot_x, self.robot_y, x, y)  # Adjust the scoring formula

            score = distance_weight * distance_score

            self.wall_frontier_scores[frontier] = score

    def score_waypoints_subsequent(self):
        self.dense_frontier_scores = {}
        for frontier in self.map_unoc_unex:
            x, y = frontier
            score = 0   

            # Adjust score higher for density of 
            distance_weight = 1.0  # Modify this weight to control how much it prioritizes proximity
            distance_score = 1 / self.euclidean_distance(self.robot_x, self.robot_y, x, y)  # Adjust the scoring formula

            score = distance_weight * distance_score

            self.dense_frontier_scores[frontier] = score

    def euclidean_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def robot_brain(self):
        current_time = time.time()

        if self.initial_exploration:
            self.score_waypoints_initial()
            self.waypoint_strategy = "furthest"
            self.initial_exploration = False
        else:
            self.score_waypoints_subsequent()
            self.waypoint_strategy = "closest"

        if self.waypoint_strategy == "furthest":
            frontier_to_visit = max(self.wall_frontier_scores, key=self.wall_frontier_scores.get)
        else:
            frontier_to_visit = min(self.dense_frontier_scores, key=self.dense_frontier_scores.get)

        x, y = frontier_to_visit

        # Create the waypoint message
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = "map"

        # If no waypoints, go to center of map
        if not self.map_callback_check:
            waypoint_msg.pose.position.x = 0.0
            waypoint_msg.pose.position.y = 0.0
        else:
            waypoint_msg.pose.position.x = x
            waypoint_msg.pose.position.y = y

        if self.patience == False:
            self.goal_publisher_.publish(waypoint_msg)

        self.last_robot_brain_time = current_time

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
