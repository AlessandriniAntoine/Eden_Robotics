# Eden_Robotics

For the ROS worskapce for the camera and the motors ros workspace see the following repository:

- camera_eden
- motors_eden

## ROS

### Create workspace

Fisrt create a workspace. Do not forget to source ROS2.

```console
~ $ mkdir -p arm_ws/src && cd arm_ws/src
~/arm_ws/src $ ros2 pkg create --build-type ament_python <package name>
~/arm_ws $ cd .. && colcon build
```

### Clone repositories

Then in the clone the motors and the camera package. Then make the workspace.

```console
~/arm_ws/src $  git clone https://github.com/AlessandriniAntoine/motors_eden.git
~/arm_ws/src $  git clone https://github.com/AlessandriniAntoine/camera_eden.git
~/arm_ws/src $ git clone https://github.com/ros-drivers/joystick_drivers.git 
~/arm_ws/src $ cd ..
~/arm_ws $ colcon build
```

### Matlab packages

Create the packages needed with the matlab files with the following command

```console
~ $ Matlab_ws/package_name/build_ros_model.sh Matlab_ws/package_name/package_name.tgz <workspace path>
```

### Launch


## Software Version

- ROS2 Foxy
- Matlab R2022a
- Python 3
- Ubuntu 20.04
- OpenCV 4.2

## Hardware Version

cf the mechanical manuel : CAO/Assembly_Plan.pdf

- dynamixels motors and driver
- Raspberry pi 3
