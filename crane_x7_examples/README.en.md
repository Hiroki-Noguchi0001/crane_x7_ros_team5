[English](README.en.md) | [日本語](README.md)

# crane_x7_examples

This package is a modified version of the original [https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples] by Chiba Institute of Technology 2020 Design and Manufacturing Theory 3, Intelligence Course Group 5.

## How to install the ROS package for CRANE-X7

```
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/RobotDesign3-Team5/crane_x7_ros_team5.git
```

---
## using the actual machine
- ### Placement of the model
  To check the operation on the actual machine, place the model as shown in the following figure.
  - Placement diagram
    <img src="" width="640px">
  - Actual layout
    <img src="https://user-images.githubusercontent.com/53966390/102636762-66c5b700-4198-11eb-89b8-87cf2d557c3d.png" width="640px">
  
- ### How to run the actual machine
  - Connect the CRANE-X7 to the USB port and change the access rights with the following command.
    ``` 
    $ sudo chmod 666 /dev/ttyUSB0
    ```
  - To check the operation, execute the following command and then run the program.
    ```
    $ roslaunch crane_x7_bringup demo.launch fake_execution:=false
    ```

---
## using gazebo
  - ### Installing the gazebo model
    Execute bash in the following way.
    ```
    $ cd ~/catkin_ws/src/crane_x7_ros_team5
    $ ./gazebo.bash
    ```
  - ### How to start gazebo
    Start gazebo with the following command.
    ```
    $ roslaunch crane_x7_gazebo crane_x7_with_table.launch
    ```

---
## Source Code
  - ### How to run
    Execute the program with the following command.
    ```
    $ roslaunch crane_x7_examples team5.launch
    ```

  - ### Sequence of events

    1.  #### greet
        - [greet.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/greet.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/greet.gif" width="320px">
      
    2.  #### artifice
        - [artifice.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/artifice.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/artifice.gif" width="320px">
      
    3.  #### Check the contents
        - [check.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/check.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/check.gif" width="320px">
      
    4.  #### Bounce off the stamp
        - [grab_release.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/grab_release.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/exclusion.gif" width="320px">
      
    5.  #### Grabbing the stamp
        - [detect_seal.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/detect_seal.py)
          - [color_detection.cpp](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/color_detection.cpp)
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/kai_detect.gif" width="320px">
      
    6.  #### Putting red ink on the stamp
        - [push_check.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/push_check.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/pushcheck.gif" width="320px">
      
    7.  #### Seal the stamp
        - [seal.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/seal.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/seal.gif" width="320px">
      
    8.  #### Wipe the stamp
        - [wipe.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/wipe.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/wipe.gif" width="320px">
      
    9.  #### Return the stamp to its original place
        - [grab_release.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/grab_release.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/release.gif" width="320px">
      
    10. #### Making a gutsy pose
        - [guts_pose.py](https://github.com/RobotDesign3-Team5/crane_x7_ros_team5/blob/master/crane_x7_examples/scripts/guts_pose.py)
        
        <img src="https://raw.githubusercontent.com/wiki/RobotDesign3-Team5/crane_x7_ros_team5/gif/gutspose.gif" width="320px">
