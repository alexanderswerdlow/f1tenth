#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group
# wget --output-document install.sh https://gist.github.com/alexanderswerdlow/b7da86e3e885d947cccba2723207a885/raw && chmod 755 install.sh && bash install.sh
# Go to update settings and change main server to mirrors.ocf.berkeley.edu
set -x -e

name_catkinws="catkin_ws"
name_ros_distro="kinetic"

version=`lsb_release -sc`

echo "[Checking the ubuntu version]"
case $version in
  "saucy" | "trusty" | "vivid" | "wily" | "xenial")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Saucy(13.10) / Trusty(14.04) / Vivid / Wily / Xenial. Exit."
    exit 0
esac

sudo sh -c "echo \"net.ipv6.conf.all.disable_ipv6 = 1\" >> /etc/sysctl.conf"
sudo sh -c "echo \"net.ipv6.conf.default.disable_ipv6 = 1\" >> /etc/sysctl.conf"
sudo sh -c "echo \"net.ipv6.conf.lo.disable_ipv6 = 1\" >> /etc/sysctl.conf"
sudo service networking restart

echo "[Update & upgrade the package]"
sudo apt -y update
sudo apt -y upgrade


echo "Your ubuntu version is $relesenum"


echo "[Installing chrony and setting the ntpdate]"
sudo apt -y install chrony ntpdate
sudo ntpdate ntp.ubuntu.com

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

echo "[Update & upgrade the package]"
sudo apt -y update
sudo apt -y upgrade

echo "[Installing ROS]"
sudo apt -y install ros-$name_ros_distro-desktop-full ros-$name_ros_distro-rqt-*

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
rosdep update
. /opt/ros/$name_ros_distro/setup.sh
sudo apt -y install python-rosinstall libarmadillo6 libarmadillo-dev git

echo "[Making the catkin workspace and testing the catkin_make]"
mkdir -p ~/$name_catkinws/src
cd ~/$name_catkinws/src

catkin_init_workspace

git clone 'https://alexanderswerdlow:3eM29&il4T@github.com/alexanderswerdlow/f1tenth.git'

cd f1tenth

git config credential.helper store

cd ~/$name_catkinws/

rosdep install --from-paths -i -y src

catkin_make

echo "[Setting the ROS evironment]"
sh -c "echo \"source /opt/ros/$name_ros_distro/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkinws/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"#export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"#export ROS_HOSTNAME=localhost\" >> ~/.bashrc"
sh -c "echo \"export ROS_MASTER_URI=http://192.168.1.1:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_IP=192.168.1.110\" >> ~/.bashrc"
sh -c "echo \"alias tf='cd /var/tmp && rosrun tf view_frames && evince frames.pdf &'\" >> ~/.bashrc"
sudo sh -c "echo \"192.168.1.1      cyber-physical\" >> /etc/hosts"

curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'

sudo apt -y update
sudo apt -y install code
code --install-extension ms-vscode.cpptools 
sudo apt -y install python-pip
python -m pip install -U "pylint<2.0.0"
code --install-extension ms-python.python
sudo apt -y autoremove
code ~/$name_catkinws/src/f1tenth/

echo "[Complete!!!]"

exec bash

exit 0
