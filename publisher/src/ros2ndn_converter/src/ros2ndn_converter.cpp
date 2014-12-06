// ros2ndn_converter.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include <ros/ros.h>

#include "ndn_controller.h"
#include "track_publisher.h"

typedef unsigned long uint32;

int
main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "ros2ndn_converter");
  ros::NodeHandle nh("~");
  ndn::NdnController::instanceStart();

  ndn::NdnController::Parameters ncParams;
  
  ncParams.nh = nh;

  // Read input parameters:
  std::string prefix, node_name;
  
  nh.param("ndn/prefix", prefix, std::string("/ndn/edu/ucla/remap"));
  nh.param("ndn/node_name", node_name, std::string("node0"));
  ncParams.prefix = ndn::NdnController::getInstancePrefix(prefix, node_name);

  nh.param("ndn/segment_length", ncParams.segmentLength, 2048);
  nh.param("ndn/daemon_host", ncParams.host, std::string("localhost"));
  nh.param("ndn/deamon_port", ncParams.port, 6363);

  ndn::opt::TrackPublisher::Parameters  tpParams;
  nh.param("ndn/freshness_period", tpParams.freshnessPeriod, 5000);
  nh.param("json/indent_size", tpParams.jsonParameters.jsonIndentSize, 0);
  nh.param("json/newline", tpParams.jsonParameters.jsonNewline, false);
  nh.param("json/spacing", tpParams.jsonParameters.jsonSpacing, false);
  nh.param("json/use_tabs", tpParams.jsonParameters.jsonUseTabs, false);
  tpParams.bufferLength = 2048;
  tpParams.nh = ncParams.nh;

  try 
  {
    ndn::NdnController ndnController(ncParams);

    if (0 == ndnController.startNdnProcessing())
    { 
      ndn::opt::ActiveTracks::Parameters atParams;
      atParams.jsonParameters = tpParams.jsonParameters;

      ndn::opt::ActiveTracks activeTracks(atParams);

      tpParams.activeTracks = &activeTracks;
      tpParams.ndnController = &ndnController; 

      ndn::opt::TrackPublisher trackPublisher(tpParams);

      trackPublisher.subscribe("input_topic");
      // ros::Subscriber sub = tpParams.nh.subscribe<opt_msgs::TrackArray>("input_topic", 1, trackingCallback);
      
      ros::spin();

      trackPublisher.unsubscribe();
      ndnController.stopNdnProcessing();      
    }
  }
  catch (std::exception& exception) 
  {
      ROS_ERROR_STREAM("got exception: " << exception.what());
  }

  return 0;
}
