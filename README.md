# Eden_Robotics

For the ROS worskapce for the camera and the motors ros workspace see the following repository:

- camera_eden
- motors_eden

## ROS

### Create workspace

Fisrt create a workspace. Do not forget to source ROS.

```console
~ $ source /opt/ros/noetic/setup.zsh
~ $ mkdir -p eden_ws/src && cd eden_ws/src
~/eden_ws/src $ catkin_init_workspace && cd ..
~/eden_ws $ catkin_make
```

### Clone repositories

Then in the clone the motors and the camera package. Then make the workspace.

```console
~/eden_ws/src $  git clone https://github.com/AlessandriniAntoine/motors_eden.git
~/eden_ws/src $  git clone https://github.com/AlessandriniAntoine/camera_eden.git
~/eden_ws/src $ cd ..
~/eden_ws $ catkin_make
```

### Matlab packages

Create the packages needed with the matlab files with the following command

```console
~ $ Matlab_ws/package_name/build_ros_model.sh Matlab_ws/package_name/package_name.tgz <workspace path>
```

Exemple :

```console
~ $ Matlab_ws/button/build_ros_model.sh Matlab_ws/button/button.tgz ~/eden_ws
```

### Launch

All the launch files are in the package camera_eden.
First open a terminal and launch ros

```console
~ $ source /opt/ros/noetic/setup.zsh && roscore
```

Open a second terminal and launch the file.

```console
~ $ cd eden_ws && source devel/setup.zsh
~/eden_ws $ roslaunch camera eden.launch
```

## Software Version

- ROS 1 noetic
- Matlab R2020b
- Python 3

## Hardware Version

cf the mechanical manuel : CAO/Assembly_Plan.pdf
