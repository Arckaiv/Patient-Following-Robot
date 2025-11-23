# Patient Following Robot
A robot capable of following a person inside a hospital

## Author
1. `Ho Xiang (A181576)`
2. `Tan Kai Ze (A202383)`
3. `Aizad Haiqal Bin Aiman Hakim SAW (A201962)`
4. `Ramnathan A/L Senthil Kumar (A203579)`

## Instructions
**LAUNCH HOSPITAL WORLD with BURGER_CAM IN GAZEBO (1ST TERMINAL)**\
`ros2 launch turtlebot3_gazebo hosp_world.launch.py`

**STOP THE HUMAN DETECTION (2ND TERMINAL)**\
`ros2 service call /yolo/toggle_detection std_srvs/srv/SetBool "{data: false}"`

**RUN RQT TO VIEW YOLO_DETECTIONS (3RD TERMINAL)**\
`rqt`
  
**START HUMAN DETECTION AND PERSON TRACKING (2ND TERMINAL)**\
`ros2 service call /yolo/toggle_detection std_srvs/srv/SetBool "{data: true}"`
