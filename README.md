# Patient Following Robot
A robot capable of following a person inside a hospital

## Author
1. `Ho Xiang (A181576)`
2. `Tan Kai Ze (A202383)`
3. `Aizad Haiqal Bin Aiman Hakim SAW (A201962)`
4. `Ramnathan A/L Senthil Kumar (A203579)`

## Requirements:
**1. Navigate to your workspace**\
`cd ~/turtlebot3_ws`

**2. Remove the cached build configuration**\
`rm -rf build/CMakeCache.txt`

**2.5. (Optional but recommended) Remove full build/install folders for a clean slate**\
`rm -rf build/ install/ log/`

**3. Update package lists**\
`sudo apt update`

**4. Install Python 3.9 and its development headers**\
`sudo apt install python3.9 python3.9-dev`

**5. Install dependencies explicitly for Python 3.9**\
`python3.9 -m pip install ultralytics`\
`python3.9 -m pip install numpy==1.24.4`\
`python3.9 -m pip install onnx onnxruntime-gpu`

**6. Navigate to source directory**\
`cd ~/turtlebot3_ws/src`

**7. Clone the autorace package (contains the camera node)**\
`git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git`

**8. Return to workspace root and build**\
`cd ~/turtlebot3_ws && colcon build --symlink-install`

**9. Source the setup file**\
`echo 'export GAZEBO_PLUGIN_PATH=$HOME/turtlebot3_ws/build/turtlebot3_gazebo:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc`

**10. Setting Turtlebot3 model (burger_cam)**\
`echo 'export TURTLEBOT3_MODEL=burger_cam' >> ~/.bashrc`

**11 Verify the camera feed**\
`ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py`

↑ Source: https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving


## Instructions
**CHANGE DIRECTORY TO TURTLEBOT3_WS (1ST TERMINAL)**\
`cd turtlebot3_ws`

**LAUNCH HOSPITAL WORLD with BURGER_CAM IN GAZEBO (1ST TERMINAL)**\
`ros2 launch turtlebot3_gazebo hosp_world.launch.py`

**STOP THE HUMAN DETECTION (2ND TERMINAL)**\
`ros2 service call /yolo/toggle_detection std_srvs/srv/SetBool "{data: false}"`

**RUN RQT TO VIEW YOLO_DETECTIONS (3RD TERMINAL)**\
`rqt`

**ROBOT SETUP (GAZEBO SIMULATION)**\
Manually drag the robot to somewhere with clear space, and drag a "person" model in the hospital map to the front of the robot, ensure the robot is able to see the person in the camera view of rqt.
  
**START HUMAN DETECTION AND PERSON TRACKING (2ND TERMINAL)**\
`ros2 service call /yolo/toggle_detection std_srvs/srv/SetBool "{data: true}"`

**ROBOT TRACKING AND FOLLOWING (GAZEBO SIMULATION)**\
Manually drag the "person" model accross the map, ensuring that model is within the camera view of the robot. Allow the robot to track the model. 
