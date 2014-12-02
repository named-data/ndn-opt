# ros2ndn_converter

**ros2ndn_converter** is a time-series publisher using NDN-CPP for OpenPTrack data. It is implemented as an ROS node of **ndn_utils** ROS package.

### Structure
Directories structure follows established [ROS guidelines](http://wiki.ros.org/ROS/Tutorials/CreatingPackage): 
 - **publisher** // _an ROS [workspace](http://wiki.ros.org/catkin/workspaces) folder_
    -  **src**
        -  **ros2ndn_converter** // _package folder_
            -  **conf** // _contains node parameters used upon launching_
            -  **include**  // _required header files_
            -  **launch**   // _[roslaunch xml file](http://wiki.ros.org/roslaunch/XML)_
            -  **src**  // _source files_
            -  CMakeLists.txt // _[catkin cmake file](http://wiki.ros.org/catkin/CMakeLists.txt)_
            -  package.xml  // _[catkin package xml file](http://wiki.ros.org/catkin/package.xml)_

### How to use
1. Clone repository
2. Build the package and make sure it is visible to ROS:
<pre>
$ cd publisher && catkin_make
$ source devel/setup.bash
</pre>
3. Now **ndn_utils** package should be listed by **rospack**:
<pre>
$ rospack find ndn_utils
&lt;repo_full_path&gt;/publisher/src/ros2ndn_converter
</pre>
4. Launch node:
<pre>
$ roslaunch ndn_utils ros2ndn_converter.launch
</pre>

> **NOTE:** in order to start publishing tracking data over NDN, make sure you have started tracking as well (by running `roslaunch tracking tracking_node.launch` from the terminal). You may also try the latest commit from [OpenPTrack NDN branch](https://github.com/OpenPTrack/open_ptrack/tree/ndn) where execution of the aforementioned command starts **ros2ndn_converter** automatically (however **ndn_utils** package has to be already installed).

### NDN Namespace

Here is the preliminary NDN namespace design used in the tracking data publisher:
![OpenPTrack data NDN publisher namespace](https://raw.githubusercontent.com/named-data/ndn-opt/master/publisher/res/namespace.png)