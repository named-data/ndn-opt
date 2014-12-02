// ros2ndn_converter.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include <ros/ros.h>
#include <opt_msgs/TrackArray.h>
#include <json.h>
#include <ndn_messaging.h>

// Constants
const std::string NdnAppComponent = "opt";	// NDN application component
						// this component is used for forming a "node" prefix:
						// <prefix>/<NdnAppComponent>/<node_name>/...
						// where prefix and node_name are supplied through
						// arguments list

// Global variables:
int ndn_segment_length; // NDN segment length - if JSON message is larger, it
			// is split into several segments
int buffer_length = 2048;
std::string prefix;	// NDN prefix used
std::string node_name;  // name of the node used while publishing NDN data
int json_indent_size;   // indent size for JSON message
bool json_newline;      // use newlines (true) or not (false) in JSON messages
bool json_spacing;      // use spacing (true) or not (false) in JSON messages
bool json_use_tabs;     // use tabs (true) or not (false) in JSON messages

ndn::opt::NDNMessaging ndn_messaging();

void
trackingCallback(const opt_msgs::TrackArray::ConstPtr& tracking_msg)
{
  ROS_INFO_STREAM("called") ;

  /// Create JSON-formatted message:
  Jzon::Object root, header, stamp;

  /// Add header (84 characters):
  header.Add("seq", int(tracking_msg->header.seq));
  stamp.Add("sec", int(tracking_msg->header.stamp.sec));
  stamp.Add("nsec", int(tracking_msg->header.stamp.nsec));
  header.Add("stamp", stamp);
  header.Add("frame_id", tracking_msg->header.frame_id);
  root.Add("header", header);

  /// Add tracks array:
  // 50 characters for every track
  Jzon::Array tracks;
  for (unsigned int i = 0; i < tracking_msg->tracks.size(); i++)
  {
    Jzon::Object current_track;
    current_track.Add("id", tracking_msg->tracks[i].id);
    current_track.Add("x", tracking_msg->tracks[i].x);
    current_track.Add("y", tracking_msg->tracks[i].y);
    current_track.Add("height", tracking_msg->tracks[i].height);

    tracks.Add(current_track);
  }
  root.Add("tracks", tracks);

  /// Convert JSON object to string:
  Jzon::Format message_format = Jzon::StandardFormat;
  message_format.indentSize = json_indent_size;
  message_format.newline = json_newline;
  message_format.spacing = json_spacing;
  message_format.useTabs = json_use_tabs;
  Jzon::Writer writer(root, message_format);
  writer.Write();
  std::string json_string = writer.GetResult();
//  std::cout << "String sent: " << json_string << std::endl;

  /// Copy string to message buffer:
  char buf[buffer_length];
  for (unsigned int i = 0; i < buffer_length; i++)
  {
    buf[i] = 0;
  }
  sprintf(buf, "%s", json_string.c_str());
//  udp_data.pc_pck_ = buf;         // buffer where the message is written

  /// Send message:
  ROS_INFO_STREAM("message sent: " << buf);
//  udp_messaging.sendFromSocketUDP(&udp_data);
}

typedef unsigned long uint32;

int
main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "ros2ndn_converter");
  ros::NodeHandle nh("~");

  // Read input parameters:
  nh.param("ndn/prefix", prefix, std::string("/ndn/edu/ucla/remap"));
  nh.param("ndn/node_name", node_name, std::string("node0"));
  nh.param("ndn/segment_length", ndn_segment_length, 2048);
  nh.param("json/indent_size", json_indent_size, 0);
  nh.param("json/newline", json_newline, false);
  nh.param("json/spacing", json_spacing, false);
  nh.param("json/use_tabs", json_use_tabs, false);

  // ROS subscriber:
  ros::Subscriber tracking_sub = nh.subscribe<opt_msgs::TrackArray>("input_topic", 1, trackingCallback);

  // Initialize NDN parameters:

  /// Create object for NDN messaging:

  // Execute callbacks:
  ROS_INFO_STREAM("start spinning");
  ros::spin();

  return 0;
}
