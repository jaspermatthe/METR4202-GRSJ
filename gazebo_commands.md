# Starter's Guide to Map
## Launch the robot and world
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

## Launch SLAM
    ros2 launch slam_toolbox online_async_launch.py
    
## Launch navigation stack
    ros2 launch nav2_bringup navigation_launch.py
this was found on https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html

## Can check available topics
    ros2 topic list

## Make sure to have made the waypoint_commander package as in Lab 4

## Copy paste the reshape_map.py file into the waypoint_commander/waypoint_commander folder

## In the setup.py of the waypoint_commander folder, make sure to have reshape_map as a console_script...
    entry_points={
            'console_scripts': [
                'waypoint_cycler = waypoint_commander.waypoint_cycler:main',
                'reshape_map = waypoint_commander.reshape_map:main',
            ],
        },

## Run this in command line from the package's folder
    colcon build --symlink-install --packages-select waypoint_commander

## MAKE SURE TO SOURCE SETUP FILE
    source install/setup.bash

## Run in command line to run the python file
    ros2 run waypoint_commander reshape_map
## OR just brute force
    python3 FULL/PATH/TO/PYTHON_FILE

## Move the bot to specified position and orientation
    ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.7, y: -0.5}, orientation: {w: 1.0}}}"

# Other Useful Commands
    ros2 topic echo /map
    ros2 run turtlebot3_teleop teleop_keyboard
    source ~/.bashrc


