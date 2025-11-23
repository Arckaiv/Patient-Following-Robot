# Patient Following Robot
A robot capable of following a perosn inside of a hospital

## Author
1. `Ho Xiang (A181576)`
2. `Tan Kai Ze (A202383)`
3. `Aizad Haiqal Bin Aiman Hakim SAW (A201962)`
4. `Ramnathan A/L Senthil Kumar (A203579)`

## Instructions
**LAUNCH HOSPITAL WORLD with BURGER_CAM IN GAZEBO**\
`ros2 launch turtlebot3_gazebo hosp_world.launch.py`

**STOP THE YOLO DETECTION**\
`ros2 service call /yolo/toggle_detection std_srvs/srv/SetBool "{data: false}"`

**RUN RQT TO VIEW YOLO DETECTIONS**\
  
**START YOLO DETECTION AND PERSON TRACKING**\
`ros2 service call /yolo/toggle_detection std_srvs/srv/SetBool "{data: true}"`
