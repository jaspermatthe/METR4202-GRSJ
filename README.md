## Team 12
Jasper Matthe 48161136
Ramtin Radnia 45890921
Gregorio Boccaccini 48175003
Sajal Gururani 47395952


## 1. clone the map_explorer repository into your ws/src folder

## in a new terminal Launch the robot and world
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

## in a new terminal Launch SLAM
    ros2 launch slam_toolbox online_async_launch.py
    
## in a new terminal Launch navigation stack
    ros2 launch nav2_bringup navigation_launch.py
this was found on https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html

## Can check available topics to see if subscriptions/publications are working
    ros2 topic list

## in a new terminal run the following in command line from the package's folder
    colcon build --symlink-install --packages-select map_explorer

## MAKE SURE TO SOURCE SETUP FILE
    source install/setup.bash

## Run in command line to run the map_explorer python file
    ros2 run map_explorer map_explorer
    
## OR just brute force
    python3 FULL/PATH/TO/PYTHON_FILE

## to visualize occupancy grid mapped by the robot, uncomment the following line in the listener_callback function:
    # plot occupancy grid
    self.plot_map()


# OTHER USEFUL COMMANDS
## Move the bot to specified position and orientation
    ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.7, y: -0.5}, orientation: {w: 1.0}}}"

## echo topics
    ros2 topic echo /map
## teleop keyboard
    ros2 run turtlebot3_teleop teleop_keyboard
## bashrc
    source ~/.bashrc

## one-liner
    gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=waffle_pi; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" -- bash -c "ros2 launch slam_toolbox online_async_launch.py" -- bash -c "ros2 launch nav2_bringup navigation_launch.py" -- bash -c "colcon build --symlink-install --packages-select map_explorer; source install/setup.bash; ros2 run map_explorer map_explorer"

-- bash -c "ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 1.7, y: -0.5}, orientation: {w: 1.0}}}'"



