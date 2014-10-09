# PCL sample
Sample usage of PCL with incoming PointCloud2 messages.

## Installation
1. Install PCL packages for ROS Hydro if you don't already have them:<br />
  `sudo apt-get install ros-hydro-pcl-conversions ros-hydro-pcl-ros ros-hydro-pcl-msgs`
2. Clone this repository:
```
cd ~/catkin_ws/src
git clone git@github.com:jstnhuang/pcl_sample.git
```

## Package creation and structure
This package was created with:<br />
`catkin_create_pkg pcl_sample pcl_conversions pcl_ros roscpp sensor_msgs`

This package is in C++. ROS packages typically put .h files in an include/packagename directory, while the .cpp files go in the src/ directory, like so:
```
pcl_sample/
pcl_sample/src/processor.cpp
pcl_sample/src/processor_node.cpp
pcl_sample/include/pcl_sample/processor.h
```

There are 3 categories of code we'll consider: libraries, executables, and tests.

A library is some piece of general code that doesn't have a main function. Executables are where the main function resides. All ROS nodes are executables. Tests are generally run against methods in libraries. Usually executables are simple enough, and delegate all their real work to library methods, so we don't usually write tests against executables. However, it's possible to start a ROS node in a unit test, and to send and receive messages to it. You can read more online about [unit testing in ROS](http://wiki.ros.org/UnitTesting).

Our library in this case is the `processor` library, `processor.h` and `processor.cpp`. Our executable, `processor_node.cpp`, is the actual ROS node that's running. In this package, we're splitting a trivial amount of code across three files just to show you the typical layout.

The 3 source files have comments describing what's going on.

catkin_create_pkg will autogenerate a CMakeLists.txt for you, but it can still be confusing. To distinguish our comments from the autogenerated documentation, our comments are preceded with four #'s: `####`

## Running

Bring up the Turtlebot by opening two windows.

In both windows, SSH into the turtlebot:<br />
```ssh turtlebot@softshell```

In one window, run:<br />
```roslaunch turtlebot_bringup minimal.launch```

In the other, run:<br />
```roslaunch turtlebot_bringup 3dsensor.launch```

Build our node:
```
cd ~/catkin_ws
catkin_make
catkin_make processor_node
```

On the lab computers, point the ROS_MASTER_URI to your Turtlebot:<br />
`setrobot softshell`

And run our node. Notice we [remap](http://wiki.ros.org/Remapping%20Arguments) the "pointcloud2_in" topic to the Turtlebot's PointCloud2 topic:
`rosrun pcl_sample processor_node pointcloud2_in:=/camera/depth_registered/points`

## Coordinate frames
The standard [ROS coordinate frame convention](http://wiki.ros.org/geometry/CoordinateFrameConventions) is
* +X forward
* +Y left
* +Z up

You can visualize this by opening up rviz on your computer:

```
setrobot softshell
roslaunch turtlebot_rviz_launchers view_robot.launch
```

In the Displays panel, click "Add", and add the "Axes" display. It should show you the default coordinate frame, where red is x, green is y, and blue is z:

![The fixed frame of the Turtlebot, obeying ROS coordinate conventions.](https://sites.google.com/site/cse481au14/labs/base_footprint.png "The fixed frame of the Turtlebot, obeying ROS coordinate conventions.")

Notice that if you run `rostopic echo /camera/depth_registered/points`, you'll see something like this:
```
header: 
  seq: ...
  stamp: 
    secs: ...
    nsecs: ...
  frame_id: /camera_rgb_optical_frame
```

This tells you that the coordinate frame for those messages is `/camera_rgb_optical_frame`. Now, in rviz, expand the "Axes" display options and choose "camera_rgb_optical_frame" for the reference frame. Notice that now, z axis points forward, the x axis points to the right, and the y axis points down.

![The coordinate frame of the PointCloud2 data, which differs ROS coordinate conventions.](https://sites.google.com/site/cse481au14/labs/camera_rgb_optical_frame.png "The coordinate frame of the PointCloud2 data, which differs ROS coordinate conventions.")

So, keep in mind that this node gives you data in a different coordinate frame from the standard ROS convention. In the future, we may want to adjust everything to use the same coordinate frame using [tf](http://wiki.ros.org/tf).
