# Pixhawk4 Vision Control

This repository explains how to integrate a Pixhawk4-controlled vehicle with Motive(TM) OptiTrack system.


## Requirements
* Windows PC running MATLAB/Simulink 2021b and [OptiTrack Motive](https://optitrack.com/software/motive/)
* Linux PC with Ubuntu 16.04 and [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Holybro S500 drone](https://shop.holybro.com/s500-v2-kit_p1153.html) with [Pixhawk4](https://shop.holybro.com/pixhawk-4_p1089.html) and [radio telemetry](https://shop.holybro.com/sik-telemetry-radio-v3_p1103.html)

## Installation
1) Install **mavros** and **mavros_extras** ROS(1) packages
```bash
  sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```
You can also install from the source [(see official guide)](https://docs.px4.io/v1.12/en/ros/mavros_installation.html#source-installation)


2) Install **vrpn_ros_client** ROS(1) packages
```bash
  sudo apt-get install ros-kinetic-vrpn-client-ros
```

3) Clone this repo in your `catkin_ws` directory
```bash
  cd ~/catkin_ws/src
  git clone https://github.com/walmaawali/px4_vision_control.git
```

4) Build the workspace
```bash
  cd ~/catkin_ws
  catkin_make
```
*Note: You could also build the workspace using `catkin build` if you installed **python-catkin-tools**

5) Source the setups to bashrc so that the `px4_vision_control` package is searchable
```bash
  sudo echo 'source ~/catkin_ws/devel/setup.bash' >> bashrc
```
Or alternatively, you could run `source ~/catkin_ws/devel/setup.bash` when opening every new terminal in Ubuntu.
> *Note: it's good idea to close and reopen every termianl when running step 4*

## Setup Drone
1) Download and open [QGroundControl](http://qgroundcontrol.com/downloads/). Install fresh firmware and perform calibaration of the drone ([see this guid](https://docs.px4.io/v1.12/en/config/firmware.html))
2) Go to  Vehicle Setup > Parameters
3) Change the following parameters to allow position estimate with external motion capture system: 

 MAVLink Parameter | Setting 
 --- | --- 
 `EKF2_AID_MASK` | Set vision position fusion, vision velocity fusion, vision yaw fusion and external vision rotation 
 `EKF2_HGT_MODE` | Set to Vision to use the vision a primary source for altitude estimation 

> *Reboot the flight controller in order for parameter changes to take effect.*

More info is found in this [guide](https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#ekf2-tuning-configuration)

## Getting Started
1) Install **mavros** and **px4_vision_control** using the steps above.
2) Power on the drone. Plug 
