#!/bin/sh
#External, ROS and system package dependencies

# ROS packages
PACKAGES="          ros-`rosversion -d`-openni-camera"
PACKAGES="$PACKAGES ros-`rosversion -d`-openni-launch"
PACKAGES="$PACKAGES ros-`rosversion -d`-orocos-kinematics-dynamics"
PACKAGES="$PACKAGES ros-`rosversion -d`-freenect-launch"
sudo apt-get install $PACKAGES

# OpenNI 1.5
cd /tmp
wget http://roboticslab.uc3m.es/mediawiki/images/c/ce/OpenNI-Bin-Dev-Linux-x86-v1.5.4.0.zip
unzip OpenNI-Bin-Dev-Linux-x86-v1.5.4.0.zip
cd OpenNI-Bin-Dev-Linux-x86-v1.5.4.0
sudo bash install.sh

# Avin (sensor)
cd /tmp
wget http://roboticslab.uc3m.es/mediawiki/images/5/54/Avin2-SensorKinect-v0.93-5.1.2.1-0-g15f1975.zip
unzip Avin2-SensorKinect-v0.93-5.1.2.1-0-g15f1975.zip
cd avin2-SensorKinect-15f1975/Platform/Linux/CreateRedist/
chmod a+x RedistMaker
sudo ./RedistMaker
cd ../Redist/Sensor-Bin-Linux-x86-v5.1.2.1
sudo chmod a+x install.sh
sudo sh install.sh

# NiTE 1.5
cd /tmp
wget http://roboticslab.uc3m.es/mediawiki/images/4/49/NITE-Bin-Dev-Linux-x86-v1.5.2.21.zip
unzip NITE-Bin-Dev-Linux-x86-v1.5.2.21.zip
cd NITE-Bin-Dev-Linux-x86-v1.5.2.21
sudo bash install.sh
