# INSTALLATION
```mkdir ur5_ws\ cd ur5_ws\ git clone -b main https://github.com/sarmadahmad8/Automated-pipeline-for-fruit-plucking-using-UR5-Robotiq-gripper-and-ZED-stereo-camera.git \ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release\ source install/setup.bash```

# DEPENDENCIES
## ENVIRONMENT DEPENDENCIES
Ubuntu 22.04\
ROS2 Humble\
CUDA 11.8

## ROS2 DEPENDENCIES
Moveit2\
moveit-visual-tools\
ros-dev-tools

## PYTHON DEPENDENCIES
Python 3.10\
CUDA 11.8\
ZED api (pyzed)\
inference-gpu==0.37.1\
opencv-python\
