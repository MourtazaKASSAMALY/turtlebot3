Original repository: https://github.com/ROBOTIS-GIT/turtlebot3/tree/eloquent-devel | See turtlebot3_onboard.patch

# turtlebot3_onboard

## Requirements:

- Ubuntu 18.04.5 LTS
- ROS2 Eloquent

## Create ROS2 Workspace:

```shell
cd ~
mkdir -p ros2_worskpace/src
echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Set proxy when connected from VPN

```bash
export http_proxy="http://defra1c-proxy.emea.nsn-net.net:8080"
export https_proxy="http://defra1c-proxy.emea.nsn-net.net:8080"
export ftp_proxy="http://defra1c-proxy.emea.nsn-net.net:8080"
echo "Acquire::http::proxy \"http://defra1c-proxy.emea.nsn-net.net:8080/\";" | sudo tee /etc/apt/apt.conf
git config --global http.proxy http://defra1c-proxy.emea.nsn-net.net:8080
```

## Build custom Turtlebot3 packages:

```shell
cd ~/ros2_workspace/src
git clone https://gitlabe2.ext.net.nokia.com/kassamal/turtlebot3_onboard.git
cd ..
colcon build --symlink-install
source ~/.bashrc
```

## Unset proxy if disconnected from VPN

```bash
export http_proxy=""
export https_proxy=""
export ftp_proxy=""
echo "Acquire::http::proxy \"\";" | sudo tee /etc/apt/apt.conf
git config --global --unset http.proxy
```

## Additional packages and configuring for Turtlebot3

```bash
sudo apt install -y ros-eloquent-dynamixel-sdk
sudo apt install -y ros-eloquent-turtlebot3-msgs
```

```bash
echo "export LC_NUMERIC=\"en_US.UTF-8\"" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

## Arduino Open CR setup

```bash
(Only for Raspberry Pi) sudo dpkg --add-architecture armhf only for Raspberry Pi
(Only for Raspberry Pi) sudo apt-get install libc6:armhf only for Raspberry Pi
sudo apt-get update

echo 'export OPENCR_PORT=/dev/ttyACM0' >> ~/.bashrc
echo 'export OPENCR_MODEL=waffle' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
source ~/.bashrc

rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xjf ./opencr_update.tar.bz2
cd ~/opencr_update
sudo chmod a+rw /dev/ttyACM0
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr

rm -r opencr_update
rm -r opencr_update.tar.bz2
```

# Usage

Bring up the robot:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

Launch the keyboard teleoperation:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

## ROS 1 Packages for TurtleBot3
|develop|master|Kinetic + Ubuntu Xenial|Melodic + Ubuntu Bionic|Noetic + Ubuntu Focal|
|:---:|:---:|:---:|:---:|:---:|
|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=develop)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=master)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=kinetic-devel)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=melodic-devel)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=noetic-devel)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|

## ROS 2 Packages for TurtleBot3
|ros2-devel|ros2|Dashing + Ubuntu Bionic|Foxy + Ubuntu Focal|
|:---:|:---:|:---:|:---:|
|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=ros2-devel)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=ros2)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=dashing-devel)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|[![Build Status](https://travis-ci.com/ROBOTIS-GIT/turtlebot3.svg?branch=foxy-devel)](https://travis-ci.com/ROBOTIS-GIT/turtlebot3)|

## ROBOTIS e-Manual for TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)

## Wiki for turtlebot3 Packages
- http://wiki.ros.org/turtlebot3 (metapackage)
- http://wiki.ros.org/turtlebot3_bringup
- http://wiki.ros.org/turtlebot3_description
- http://wiki.ros.org/turtlebot3_example
- http://wiki.ros.org/turtlebot3_navigation
- http://wiki.ros.org/turtlebot3_slam
- http://wiki.ros.org/turtlebot3_teleop

## Open Source related to TurtleBot3
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
- [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
- [turtlebot3_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
- [turtlebot3_deliver](https://github.com/ROBOTIS-GIT/turtlebot3_deliver)
- [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)
- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_simulations)
- [open_manipulator_perceptions](https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
- [open_manipulator_with_tb3_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_msgs)
- [open_manipulator_with_tb3](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3)
- [open_manipulator_with_tb3_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_simulations)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [ROBOTIS e-Manual for Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
- [Website for TurtleBot Series](http://www.turtlebot.com/)
- [e-Book for TurtleBot3](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for TurtleBot3 ](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)
