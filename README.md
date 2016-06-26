# `kinect`

`kinect` is a [ROS](http://www.ros.org/) package that
provides nodes to acquire information
from a Microsoft Kinect device.

[![Build Status](https://travis-ci.org/UC3MSocialRobots/kinect.svg)](https://travis-ci.org/UC3MSocialRobots/kinect)

How to install
==============

Dependencies: please run the ```rosdep``` utility:

```bash
$ sudo apt-get install python-rosdep
$ sudo rosdep init
$ rosdep install kinect
```

Drivers
=======

There are four drivers (primitives) that can be used with the Kinect.

* the [***freenect***](http://wiki.ros.org/freenect_launch) driver
  * It is open source
  * It does not consume a lot

* the [***pure openni 1***](http://wiki.ros.org/openni_camera) driver, integrated in ROS
  * It gives you the RGB and the depth image, but not the user mask.
  * It is open source
  * It does not consume a lot

* the [***pure openni 2***](http://wiki.ros.org/openni2_launch) driver, integrated in ROS
  * very similar to OpenNI1

* the [***OpenNI+NITE***](http://www.openni.org/files/nite/ NITE page]) driver
  * It gives you the RGB and the depth image, and the user map (that is a nice feature!)
  * It also publishes the skeleton, both as
    a set of ROS TF transforms (http://wiki.ros.org/tf ROS TF page])
    and a NiteSkeletonList custom message (https://163.117.150.59/browser/repoAD/projects/devices/kinect/unstable_ros/msg/NiteSkeletonList.msg msg file])
  * It is closed source (at least the low level NITE engine)
  * It does not consume a lot

Which should I need?
====================

It depends what you need. If you need the user map, go for NITE. Otherwise, use the freenect/openni one.

What topics do they supply?
===========================

They offer a unified interface:

* ```/<robot>/rgb``` [sensor_msgs/Image]
* ```/<robot>/depth``` [sensor_msgs/Image]
* ```/<robot>/user``` [sensor_msgs/Image] (only for NITE)
* ```/<robot>/skeletons``` [kinect/NiteSkeletonList] (only for NITE)
* ```/<robot>/cur_tilt_angle``` [std_msgs/Float64]
* ```/tf``` [tf/tfMessage]

Code API
========

Cf class ```NitePrimitiveClass```
and both implementations
```nite_primitive_standalone.cpp``` and ```nite_primitive.cpp```
