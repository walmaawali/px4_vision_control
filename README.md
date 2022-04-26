# Pixhawk4 Vision Control

This repository explains how to integrate a Pixhawk4-controlled vehicle with OptiTrack Motive system.

[![BSD License](https://img.shields.io/badge/license-BSD-green.svg)](https://github.com/walmaawali/px4_vision_control/blob/main/LICENSE)
[![Python 2.7](https://img.shields.io/badge/python-2.7-blue.svg)](https://www.python.org/download/releases/2.7/)
[![ROS Kinetic](https://img.shields.io/badge/ros-kinetic-red.svg)](http://wiki.ros.org/kinetic)
[![Ubuntu 16.04 LTS](https://img.shields.io/badge/ubuntu-16.04-orange.svg)](https://releases.ubuntu.com/16.04/)
[![MATLAB 2021b](https://img.shields.io/badge/matlab-2021b-lightgrey.svg)](https://www.mathworks.com)

## Architecture
### Software Architecture
<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/software_architecture.jpg" width="600" />

### Hardware Architecture
<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/hardware_architecture.jpg" width="600" />

## Requirements
* Windows PC running MATLAB/Simulink 2021b and [OptiTrack Motive](https://optitrack.com/software/motive/)
* Linux PC with Ubuntu 16.04 and [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Holybro S500 drone](https://shop.holybro.com/s500-v2-kit_p1153.html) with [Pixhawk4](https://shop.holybro.com/pixhawk-4_p1089.html) and [radio telemetry](https://shop.holybro.com/sik-telemetry-radio-v3_p1103.html)
* USB WiFi adapter (we used [TP-Link AC1900](https://www.amazon.com/TP-Link-Archer-T9UH-Wireless-network/dp/B01GE9QS0G/)

> Note: The USB WiFi adapter is not needed if the Linux and Windows PC exist in the same network (i.e. connected to an ethernet switch)

> Hint: You may run a Linux virtual machine within the Windows PC and perform the steps below without needing another Linux PC. Make sure that network is well configured, and USB devices are enabled on the virtual machine.

## Installation
1) Install **mavros** and **mavros_extras** ROS(1) packages[^1]
```bash
  sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

2) Install **vrpn_ros_client** ROS(1) packages
```bash
  sudo apt-get install ros-kinetic-vrpn-client-ros
```

3) Clone this repo in your `catkin_ws` directory
```bash
  cd ~/catkin_ws/src
  git clone https://github.com/walmaawali/px4_vision_control.git
```

4) Build the workspace[^2]
```bash
  cd ~/catkin_ws
  catkin_make
```

5) Source the setups to bashrc so that the `px4_vision_control` package is searchable[^3]
```bash
  sudo echo 'source ~/catkin_ws/devel/setup.bash' >> bashrc
```

> *It's good idea to close and reopen all termianls when running step 5*

### Testing the integration
1) Power on the drone
2) Plug the USB telemetry radio. On the termial, enable access to the device
```bash
  sudo chmod 666 /dev/ttyUSB0
```
> Note: the USB telemetry radio may be on a different directoty (i.e. `/dev/ttyUSB1` or `/dev/ttyACM0`). Run `ls /dev` to see all serial devices. Try unplug and replug the USB telemetry radio and see which file gets created.

3) Launch mavros node
```bash
  roslaunch mavros px4.launch
```
4) Open another terminal, and issue arming command
```bash
  rosrun mavros mavsafety arm
```
5) To disarm, use the command
```bash
  rosrun mavros mavsafety disarm
```
> Note: issuing the arming command may be rejected. If the drone produces a "rejection" sound, the integration still is successful. The sound means that not all arming conditions are satisfied. You may need to stream motion capture position (more into that later).


## Setup Drone for External Position Estimate
1) Download and open [QGroundControl](http://qgroundcontrol.com/downloads/). 
2) Install fresh firmware and perform calibaration of the drone. See this [guide](https://docs.px4.io/v1.12/en/config/firmware.html)
3) In QGroundControl, go to  Vehicle Setup > Parameters
4) Change the following parameters to allow position estimate with external motion capture system: 

 Parameter | Setting 
 --- | --- 
 `EKF2_AID_MASK` | Set vision position fusion, vision velocity fusion, vision yaw fusion and external vision rotation 
 `EKF2_HGT_MODE` | Set to Vision to use the vision a primary source for altitude estimation 

> *Reboot the flight controller in order for parameter changes to take effect.*

> More info about the setup can found in this [reference](https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#ekf2-tuning-configuration)

## Setup OptiTrack Motive
1) Perform setup of the motion capture system. See this [guide](https://v30.wiki.optitrack.com/index.php?title=Quick_Start_Guide:_Getting_Started)
2) Perform calibration of the system. See this [guide](https://v30.wiki.optitrack.com/index.php?title=Calibration)
3) Place (at least three) markers on the drone.

<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/drone.jpg" height="500" />

4) Place the drone in the arena. You should see the markers in Motive.
5) Select the markers of the drone (at least three) and right-click in Motive and select `Rigid Body -> Create From Selected Markers`

<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/create_rigid_body.png" width="500" />

6) Open the asset pane click on `View -> Asset Pane`.

<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/asset_pane.png" width="500" />

7) The rigid body will appear in `Assets` in the project pane. Rename the drone to `drone1`.

<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/motive_asset_pane.PNG" />

> Hint: If another name is desired, use a name without a space (for example, use `robot_1` or `robot1` instead of `robot 1`).

> Keep a note on the name of the drone, as it will be used in ROS.

8) Open the data streaming pane click on `View -> Data Streaming Pane`.

<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/streaming_pane.png" width="500"/>

9) Enable VRPN broadcasting and Frame broadcasting according to the settings below (you need to show advanced settings to see all the fields). Make sure VRPN broadcating port is **3883**.

<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/motive_streaming_settings.PNG" height="500"/>

10) Start live streaming by clicking the red dot in OptiTrack Motive

### Testing the integration

> Note: The following steps are illusterated for **WiFi connection** between the Linux PC and Windows PC. You may use ehternet connection, ehternet switch, or virtua machine as mentioned earlier.

1) Place the drone in the arean.
2) Connect the **USB WiFi adapter** to Windows PC, and enable [mobile hotspot](https://support.microsoft.com/en-us/windows/use-your-windows-pc-as-a-mobile-hotspot-c89b0fad-72d5-41e8-f7ea-406ad9036b85#WindowsVersion=Windows_10) from the network settings.
3) Connect the Linux PC to the mobile hotspot. Keep a note on the IP addresses of the Windows PC and Linux PC by running this command in Linux terminal
```bash
  ifconfig
```

4) After perfoming Installation steps, run the following command on Linux terminal  (change the server IP address `192.168.137.1` to the one obtained from step 3)
```bash
  roslaunch vrpn_client_ros sample.launch server:=192.168.137.1
```

5) To check that motion data is received by ROS, open another termianl and run
```bash
  #running this command should show a new topic /vrpn_client_node/robot1/pose
  rostopic list 

  # you shoud see a stream of data after running this command
  rostopic echo /vrpn_client_node/robot1/pose
```

Here we have named our rigid body `robot1`. The name may be different according to the name you have chosen while creating the rigid body.

> Note: streaming will stop if you take out the drone from the arena.

6) (Optional) To visuaize data in ROS, run the following commands, each one in a seperate terminal
```bash
  #convert from OptiTrack coordinate system to ROS coordinate system
  rosrun tf static_transform_publisher 0 0 0 1.57 0 1.57 map world 10

  # visuaizatin tool in ROS
  rviz
```

Next, add a *Pose* and *tf* objects in RViZ, according to the image below. The object in RViZ should move as you move or rotate the drone in the arena.

<img src="https://github.com/walmaawali/px4_vision_control/blob/main/images/rviz.jpg" width="500"/>

> More info can be found in this [reference](https://tuw-cpsg.github.io/tutorials/optitrack-and-ros/). See VRPN section.

## References
[OptiTrack Motive](https://v30.wiki.optitrack.com/index.php?title=OptiTrack_Documentation_Wiki)

[VRPN ROS Package](http://wiki.ros.org/vrpn_client_ros)

[MAVROS](http://wiki.ros.org/mavros)

[How to setup motion capture with ROS](https://tuw-cpsg.github.io/tutorials/optitrack-and-ros/)

[How to setup PX4 for motion capture](https://docs.px4.io/v1.12/en/ros/external_position_estimation.html#ekf2-tuning-configuration)

## Authors and Contributers
- [Dr. Jawhar Ghommam](https://www.researchgate.net/profile/Jawhar-Ghommam)
- [Ibrahim Al Jahwari](https://github.com/Ibrahim9955)
- [Waleed Al Maawali](https://www.github.com/walmaawali)
- [Madona Ibrahim]()
- [Majid Al Mujaini](https://github.com/Mujaini-M)

## Footnotes
[^1]:You can also install from the source using this [guide](https://docs.px4.io/v1.12/en/ros/mavros_installation.html#source-installation)

[^2]: You could also build the workspace using `catkin build` if you installed **python-catkin-tools**

[^3]: Or alternatively, you could run `source ~/catkin_ws/devel/setup.bash` when opening every new terminal in Ubuntu.





