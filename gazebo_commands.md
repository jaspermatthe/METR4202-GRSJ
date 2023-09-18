# Starter's Guide to Map
## 1. Launch the robot and world
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

## 2. Launch SLAM and navigation stack
    ros2 launch slam_toolbox online_async_launch.py
    ros2 launch nav2_bringup navigation_launch.py
this was found on https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html

## 3. Can check available topics
    ros2 topic list

## 4. Make sure to have made the waypoint_commander package as in Lab 4

## 5. Copy paste the reshape_map.py file into the waypoint_commander/waypoint_commander folder

## 6. in the setup.py of the waypoint_commander folder, make sure to have reshape_map as a console_script...
    entry_points={
            'console_scripts': [
                'waypoint_cycler = waypoint_commander.waypoint_cycler:main',
                'reshape_map = waypoint_commander.reshape_map:main',
            ],
        },

## 7. Run this in command line:
    colcon build --symlink-install --packages-select waypoint_commander

## 8. Run in command line to run the python file
    ros2 run waypoint_commander reshape_map
## 8. OR just brute force
    python3 FULL/PATH/TO/PYTHON_FILE

## 9. Move the bot to specified position and orientation
    ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.7, y: -0.5}, orientation: {w: 1.0}}}"

# Other Useful Commands
    ros2 topic echo /map
    ros2 run turtlebot3_teleop teleop_keyboard
    source ~/.bashrc


