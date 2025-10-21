
Requirements
------------

* To build and install RosAria, first install the ARIA library. You can either download it from [this site](https://web.archive.org/web/20180214114520/http://robots.mobilerobots.com/wiki/Main_Page) or AriaCode [from here](http://github.com/reedhedges/AriaCoda) (recommended).
* Make sure that the libaries are included in the same path as the CMakeLists.txt.

Dependencies
------------

  * [tf2_ros](https://github.com/ros2/geometry2/tree/ros2/tf2_ros).
  * [tf2](https://docs.ros.org/en/foxy/Tutorials/tf2.html).
  * std_msgs, std_srvs, sensor_msgs, nav_msgs and geometry_msgs


Notes
-----------------------------------------
* The config files has been removed, and most of the parameters are hard coded within the RosAria.cpp constructor. 
* You might need to change the serial_port, if yours are different from the one already set.


### Install RosAria
```bash
#get code
mkdir -p ~/pioneer/src
cd ~/pioneer/src
git clone https://github.com/David1340/RosAria2.git

#build
cd ~/pioneer
source /opt/ros/humble/local_setup.bash
colcon build 
```

## Usage Instructions

### Start the RosAria node
Set-up the robot, then type the following command:

```bash
source /opt/ros/foxy/setup.bash
source ~/pioneer/install/local_setup.bash
# To launch with "ros2 run"
ros2 run rosaria RosAria
```

This will start the robot and publish on the required ROS2 topics.

