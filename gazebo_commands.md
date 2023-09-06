# When making new worlds, run this in terminal
jasper@SuperDuperSurface:~/turtlebot3_ws$ colcon build --symlink-install

# To launch your world with rviz slam mapping:
1. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
2. ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
map:=/home/jasper/Documents/METR4202/maps/turtlebot3_world_map.yaml
3. provide robot pose estimate manually
4. ros2 topic echo /map
3. ros2 run turtlebot3_teleop teleop_keyboard
4. export TURTLEBOT3_MODEL=waffle_pi && \



# Misc.
source ~/.bashrc


# To move the bot to specified position and orientation
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id:
'map'}, pose: {position: {x: 1.7, y: -0.5}, orientation: {w: 1.0}}}"

# 