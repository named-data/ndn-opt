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
Sample track data received from OpenPTrack:
<pre>
{"header":{"seq":2614,"stamp":{"sec":1417543531,"nsec":207120105},"frame_id":"world"},"tracks":[{"id":4,"x":3.79707,"y":2.75168,"height":1.43847}]}
</pre>

The following assumptions were taken before designing a namespace:
1. Track data has _nanosecond_-precision timestamps
2. Track data arrives at non-constant rate
3. Track data is very lightweight (around 80-100 bytes)

Given that and the fact that NDN overhead may be about the size of the payload or even bigger, publishing individual tracks under separate names would be inefficient. Good solution for that would be the bundling of tracks.

Another design decision was made in favor of using sequential numbering rather than timestamps. Tracking data is timestamped using nanosecond precision which can not easily mapped into NDN namespace. One should either concatenate "seconds" timestamp with "nanoseconds" timestamp, thus obtaining unique name component (which may take up to 20 symbols) or nest nanoseconds under seconds timestamps in the namespace and make consumer's fetching algorithms more complicated. Another motive of using sequential numbering is that it allows consumers to keep control over the number of outstanding interests (as they know that numbers are incremented sequentially unlike timestamps).

Proposed namespace:
![OpenPTrack data NDN publisher namespace](https://raw.githubusercontent.com/named-data/ndn-opt/master/publisher/res/namespace.png)

### Payloads
There are three types of payloads:
- Track payload
    - Contains tracking data bundle which is represented as Json array. Each array element contains timestamps for the given data and coordinates. Published at 30-60Hz rate.
        <pre>
{
"tracks": [{
	        "sec":1417543531,
	        "nsec":40461546,
	        "x":3.33293,
	        "y":2.67657,
	        "z":1.3314
            },
            ...]
}
        </pre>
- Hint payload
    - Used for efficient bootstrapping. This data is published with 1-5Hz rate. Active tracks array contains only those tracks for which new data was received during the last refreshing period.
        <pre>
        {
            "active tracks": [1,2,3,5,8]
        }
        </pre>
- Meta payload
    - Contains any meta information about current node (calibration info, etc.)

### Publisher
Here is the preliminary app design which realizes publishing in the aforementioned namespace:
![OpenPTrack data NDN publisher design](https://raw.githubusercontent.com/named-data/ndn-opt/master/publisher/res/ndn-opt-publisher.png)

Each track received from OpenPTrack is analyzed for the track id and tracking info (timestamps and coordinates). This tracking info is extracted, packed into "ready-to-be-published" Json object and placed in the **track bucket**. Track buckets are synchronization objects (protected with mutexes) which allow ony one thread at a time to modify them. There is a separate bucket for each track id. Track bucket get filled by a process which receives ROS callbacks. Simultaneously, bucket is emptied every time when **track publisher** get access to the bucket. At this moment, track publisher packs all tracks from the bucket into a Json object, publishes this object to NDN and empties the bucket. Track publisher operates on the constant rate (configurable).
The other instance which gets access (read-only) to the buckets array is the **track hint publisher** which also operates on the constant rate and publishes track hints to NDN. Track hint publisher may also delete empty buckets.

