# Map Explorer with ROS for Team 12
Welcome to the Map Explorer project for Team 12! This README provides instructions on how to set up and run our code and exploration strategy.    
Jasper Matthé 48161136  
Ramtin Radnia 45890921  
Gregorio Boccaccini 48175003  
Sajal Gururani 47395952  

## 1. Clone the Repository
Firstly, clone the 'map_explorer' repository into your ROS 2 workspace 'src' folder:
```
cd ~/YOUR/ROS2/WORKSPACE/src
git clone https://github.com/jaspermatthe/METR4202-GRSJ/tree/f92455438588bd3292fd97b4614dac9fcffcc701/map_explorer
```

## 2. Build the Package
In a new terminal, navigate to your workspace and build the map_explorer package:
```
colcon build --symlink-install --packages-select map_explorer
```

After building, remember to source your workspace:
```
source install/setup.bash
```
    
## 3. Launch the Robot and Environment
In a new terminal, set the wanted robot model (waffle_pi in this case) and launch it in the - preferred - simulated world:
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## 4. Launch SLAM
In a new terminal, initiate SLAM:
```
ros2 launch slam_toolbox online_async_launch.py
```
    
## 5. Launch Navigation Stack
In a new terminal, start the navigation stack:
```
ros2 launch nav2_bringup navigation_launch.py
```  
(this was found on https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)

## Check available topics to see if subscriptions/publications are working
```
ros2 topic list
```

## 6. Run Map Explorer
Execute the map_explorer script:
```
ros2 run map_explorer map_explorer
```
    
Alternatively, you can use:
```
python3 FULL/PATH/TO/MAP_EXPLORER_PYTHON_FILE
```

# Additional Commands
Here are some helpful commands you might use during the development and testing phases:

## Move the bot to specified position and orientation:
```
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.7, y: -0.5}, orientation: {w: 1.0}}}"
```

## Echo topics:
```
ros2 topic echo /map
```
## Keyboard teleoperation:
```
ros2 run turtlebot3_teleop teleop_keyboard
```
## Source .bashrc:
```
source ~/.bashrc
```

## One-liner to execute multiple commands:
```
gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=waffle_pi; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" -- bash -c "ros2 launch slam_toolbox online_async_launch.py" -- bash -c "ros2 launch nav2_bringup navigation_launch.py" -- bash -c "colcon build --symlink-install --packages-select map_explorer; source install/setup.bash; ros2 run map_explorer map_explorer"
```




