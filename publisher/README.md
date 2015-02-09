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

### Prerequisites

1. [NDN-CPP library](https://github.com/named-data/ndn-cpp)

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
4. Add shared library path to ROS environmental variable:
<pre>
    $ export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
</pre>
5. Launch node:
<pre>
$ roslaunch ndn_utils ros2ndn_converter.launch
</pre>

> **NOTE:** in order to start publishing tracking data over NDN, make sure you have started tracking as well (by running `roslaunch tracking tracking_node.launch` from the terminal).

### NDN Namespace
Sample track data received from OpenPTrack:
<pre>
{
    "header":{
        "seq":2614,
        "stamp": {"sec":1417543531,"nsec":207120105},
        "frame_id":"world"
    },
    "tracks":[{"id":4,"x":3.79707,"y":2.75168,"height":1.43847}]
}
</pre>

The following assumptions were taken before designing a namespace:
1. Track data has _nanosecond_-precision timestamps
2. Track data arrives at non-constant rate
3. Track data is very lightweight (around 80-100 bytes)

Given that and the fact that NDN overhead may be about the size of the payload or even bigger, publishing individual tracks under separate names would be inefficient. A solution for that would be the bundling of tracks. However, in this way, consumer has to provide complex algorithms for re-bundling samples, buffer and sort them and retrieve the latest. Therefore, we'd stick with non-bundling approach, chosing simpler and faster implementation rather then optimized data transfering.

Another design decision was made in favor of using sequential numbering rather than timestamps. Tracking data is timestamped using nanosecond precision which can not easily mapped into NDN namespace. One should either concatenate "seconds" timestamp with "nanoseconds" timestamp, thus obtaining unique name component (which may take up to 20 symbols) or nest nanoseconds under seconds timestamps in the namespace and make consumer's fetching algorithms more complicated. Another motive of using sequential numbering is that it allows consumers to keep control over the number of outstanding interests (as they know that numbers are incremented sequentially unlike timestamps).

Sequential numbering is preferrable for real-time consumers. However, there is another class of historical consumers, which operate using timestamps and perform history seeking. In order to support this, timestamps as name components were chosen for track hints. **offset_msN** represents a milliseconds-precision offset from the **instance_start_time_sec**. In this way, historical consumers can easily perform seeking and retrieving tracking samples using IDs stored in hint's payloads.

Proposed namespace:
![OpenPTrack data NDN publisher namespace](https://raw.githubusercontent.com/named-data/ndn-opt/master/publisher/res/namespace.png)

### Payloads
There are three types of payloads:
- Track payload
    - Contains tracking data which is represented as Json object. Object contains timestamps for the given data and coordinates. Published at the incoming rate of the tracking data. 
        <pre>
{
    "sec":1417543531,
    "nsec":40461546,
    "x":3.33293,
    "y":2.67657,
    "z":1.3314
}
        </pre>
        - **sec** - seconds-precision timestamp
        - **nsec** - nanoseconds-precision timestamp
        - **x, y, z** - tracking sample coordinates
- Hint payload
    - Used for efficient bootstrapping. This data is published with 1-5Hz rate. Tracks array contains only those tracks for which new data was received during the last refreshing period.
<pre>
{ 
        "sec":1417543531, 
        "nsec":40461546, 
        "tracks": [ {"id":1, "seq":42}, {"id":2, "seq":12}, ...]
}
</pre>
        - **sec, nsec** - seconds- and nanoseconds-precision timestamp 
        - **tracks** - array of active tracks; each element contains:
            - **id** - track id which can be used for querying track payload
            - **seq** - latest track's sample sequence number which can be used for querying track payload
- Meta payload
    - Contains any meta information about current node (calibration info, etc.)
<pre>
Meta payload TBD
</pre>

### Publisher
Here is the preliminary app design (Version 2, you may also check [Version 1 design](https://raw.githubusercontent.com/named-data/ndn-opt/master/publisher/res/ndn-opt-publisher.png)) which realizes publishing in the aforementioned namespace:
![OpenPTrack data NDN publisher design](https://raw.githubusercontent.com/named-data/ndn-opt/master/publisher/res/ndn-opt-publisher_v2.png)

Each track received from OpenPTrack is analyzed for the track id and tracking info (timestamps and coordinates). This tracking info is extracted, packed into "ready-to-be-published" Json object and published by **Track publisher**. Besides publishing, track publisher updates **Active Tracks** structure with the most recent data - recently received track id, sequence number name component and timestamps (seconds and nanoseconds). Active Tracks structure is a synchronization object - access to it is mutually exclusive for all threads. Active Tracks structure is invalidated every time when **Track hint publisher** gets access to it. Track hint publisher operates at the constant rate and publishes track hints to NDN. Track hint publisher does not publish any data if Active Tracks has no new data to publish.

