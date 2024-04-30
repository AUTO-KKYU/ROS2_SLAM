# ROS2_SLAM
Minibot navigation SLAM , Waypoint setting in python code  

### Execute All in workspace
1) Build and Execute Gazebo
```sh
$ colcon build
# alias 명령어
$ my_mobile
$ ros2 launch my_robot_gazebo launch_sim.launch.xml
```
2) Execute Navigation based on implemented map
```sh
$ my_mobile
$ ros2 launch my_robot_navigation bringup_launch.xml use_sim_time:=true map:=map_name.yaml
```

3) Display env in Rviz
```sh
$ my_mobile
$ ros2 launch my_robot_navigation nav2_view.launch.xml
```
<img src= "https://github.com/AUTO-KKYU/ROS2_SLAM/assets/118419026/1c30079f-8757-4a56-82ec-34e892505cd3">

- Check Topic list
```sh
$ my_mobile
$ ros2 topic list
```

- You can find out the current position coordinates of the robot by subscribing to this topic [/amcl_pose]
```sh
$ ros2 topic echo /amcl_pose
```

- If you look at the terminal screen where you subscribed to the topic, you will see the coordinates of the clicked location
- Click Publish Point Button in Rviz and click anywhere in map and watch the result in terminal
```sh
$ ros2 topic echo /clicked_point 
```

- waypoint setting based on clicked_point position and run the code 
```sh
$ my_mobile
$ ros2 run my_robot_test navigate_robot
```
https://github.com/AUTO-KKYU/ROS2_SLAM/assets/118419026/ab518f3c-3859-4c2c-bd1c-4272132b6a34


