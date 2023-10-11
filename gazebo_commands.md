# Starter's Guide to Map
## Create a new "map_explorer" package as in Lab 4:
### cd to your workspace (change /path/to/your/ accordingly)
    cd path/to/your/ros_ws/src
### create package
    ros2 pkg create --build-type ament_python map_explorer --dependencies rclpy nav2_msgs geometry_msgs
    
### cd to inner package map_explorer folder (change /path/to/your/ accordingly)
    cd path/to/your/ros_ws/src/map_explorer/map_explorer
    
### make map_explorer python file
    touch map_explorer.py

## Copy paste all the code in the "map_explorer_jasper.py" https://github.com/jaspermatthe/METR4202-GRSJ/blob/main/map_explorer_jasper.py  file into your newly created "map_explorer.py" file and save it

## In the setup.py of the map_explorer folder, make sure to have map_explorer as a console_script...
    entry_points={
            'console_scripts': [
                'map_explorer = map_explorer.map_explorer:main',
            ],
        },


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

## in a new terminal Run the following in command line from the package's folder
    colcon build --symlink-install --packages-select map_explorer

## MAKE SURE TO SOURCE SETUP FILE
    source install/setup.bash

## Run in command line to run the map_explorer python file
    ros2 run map_explorer map_explorer
    
## OR just brute force
    python3 FULL/PATH/TO/PYTHON_FILE

# OTHER USEFUL COMMANDS
## Move the bot to specified position and orientation
    ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.7, y: -0.5}, orientation: {w: 1.0}}}"

## echo topics
    ros2 topic echo /map
## teleop keyboard
    ros2 run turtlebot3_teleop teleop_keyboard
## bashrc
    source ~/.bashrc


