# Autonomous Aerial Recovery System of a Compound Mother-Daughter UAV System

Created by Xuyi FU from Beihang University.

### Abstract
The recovery of micro aerial vehicles (MAVs) is key in mother-daughter robotic cooperation. This paper aims to develop a recovery system for mother-daughter UAV system, which consists of a recovery mechanism subsystem and its corresponding recovery algorithms. The UAV recovery system relies on a finite state machine architecture, and a relocation state is included for the daughter-UAVs to relocate the corresponding recovery platform. The recovery mechanism is designed as a shelter, which includes three layers of the rolling-shutter platform and one layer of the bottom platform. The developed algorithm includes the part of detection and the tracking controller. The detection subsystem based on a multi-state visual marker is developed, which helps the daughter-UAVs locate the recovery platform quickly and accurately. Then, a state-based PID controller and a reference-speed compensation controller are developed and tested for the tracking subsystem, in order to transform a "moving platform" recovery problem into a "static platform" recovery problem. The proposed recovery system is tested in the simulation environment (Gazebo), in which the mother-UAV’s motion mode includes static and moving. The results show that the state-based PID controller with reference-speed compensation produces a better tracking effect, the proposed system can realize the autonomous recovery of four UAVs in aerial.

The above is the summary of the paper, but this program is for one mother-UAV and one daughter-UAV's docking simulation. The  four mother-UAVs and four daughter-UAVs' docking simulation program will be collated and transmitted later.

![2021-07-08 21-44-55 的屏幕截图](/home/fxyttql/Pictures/2021-07-08 21-44-55 的屏幕截图.png)

We have tested our system on ROS Melodic and Ubuntu 18.04.

You can follow [this](http://wiki.ros.org/melodic/Installation/Ubuntu) tutorial to install ROS Melodic.

You can follow [this](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#rosgazebo) tutorial to build PX4-Firmware and other required functions like Gazebo and mavros. We recommend that you replace the Firmware after configuring Px4 environment according to the official tutorial.


## 2. Usage

First of all, create a catkin workspace and move into it. Then clone this repository into a folder (e.g., `src`). We'll install some ROS dependencies and, finally, go back to `ws`, build the workspace and source your environment. The individual steps are:

```bash
$ mkdir docking_ws && cd docking_ws
$ git clone git@github.com:fxyonlyone/mother-daughter-UAVs-autunomous-aerial-docking.git src
```

Install all required ROS packages for this project by running:

```bash
$ cd src
$ chmod +x install_dependencies.sh
$ ./install_dependencies.sh
```

Next, cut the Firmware file to the home directory, then Compile and configure the PX4 environment:

```bash
$ sudo mv Firmware/ /home/ubuntu
$ cd
$ cd Firmware
$ make px4_sitl_default gazebo
```

Finally, compile the project and source your environment:

```bash
$ cd
$ cd docking_ws
$ catkin_make
$ source devel/setup.bash
```

Now launch the world with both the mother-daughter UAVs:

```bash
$ roslaunch px4 multi_uav_mavros_sitl_couple.launch 
```

In a different terminal (don't forget to source the environment everytime you open a new terminal by using `cd docking_ws && source devel/setup.bash`), launch the detection and tracking modules:

```bash
$ roslaunch duav_dock both_takeoff_dock.launch
```

Finally, also in a different terminal (don't forget `source devel/setup.bash`), launch the Kalman prediction module (Kalman filter is not fully developed, so this step can be ignored):

```bash
$ roslaunch ped_traj_pred kalman_pred.launch
```

After starting the both_takeoff_dock.launch, the mother-daughter UAVs will fly to the specified altitude. Now you can press the keyboard "d" or "D", and the daughter-UAV will fly to the mother UAV and start tracking and docking. This is the default mission, but feel free to design your own!

## 3. Supplementary notes

1. Because the mother-UAV's model is not built well enough, it will take off and move, but it will be ready in a moment.

2. Parameter log_dir in dUAVDock.h needs to be changed to your corresponding.
3. Parameter mUAV_moving in mUAVTakeOff.cpp can be "True" or "False" to determine the mother-UAV move or hover.

4. The third generation AprilTag method can directly solve the yaw angle, which is written in track.py, but the angular velocity is not used in the actual tracking.
