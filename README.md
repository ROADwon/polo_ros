## Polo(Polygon - zone Yolo Detection)

## Description
### - 1. This package was build for object detection in polygon zone.

---

## Build Env
### - 1. Python 3.8 
### - 2. ROS2 FOXY (ros2 distortion)
###  - 3. Jetson Jetpack 4.5.1

---

## Installation and Quick Start
### - Install the package :
    mkdir  ${project_name}_ws/src

    cd ${project_name}_ws/src

    git clone "https://github.com/ROADwon/polo.git"
    
    cd ${project_name}_ws

    colcon build --symink-install

    . install/setup.bash
    
    ros2 launch yolo_ros polo_launch.py

---

## More...

### - if you change the polygon vector you can change "points" in polo_pt.py file.
### - if you have another camera topic you can change topic  line 51 in polo_pt.py file. 