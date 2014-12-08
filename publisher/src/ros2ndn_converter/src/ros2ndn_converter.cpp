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
#include "track_hint_publisher.h"

typedef unsigned long uint32;

int
main(int argc, char **argv)
{
  // initialization:
  ros::init(argc, argv, "ros2ndn_converter");
  ros::NodeHandle nh("~");

  // declare instance start 
  ndn::NdnController::instanceStart();

  // read Json parameters from node parameters file
  JsonParameters jsonParameters;
  nh.param("json/indent_size", jsonParameters.jsonIndentSize, 0);
  nh.param("json/newline", jsonParameters.jsonNewline, false);
  nh.param("json/spacing", jsonParameters.jsonSpacing, false);
  nh.param("json/use_tabs", jsonParameters.jsonUseTabs, false);

  try 
  {
    // setting up NdnController
    ndn::NdnController::Parameters ncParams;
    ncParams.nh = nh;

    std::string prefix, node_name;
  
    nh.param("ndn/prefix", prefix, std::string("/ndn/edu/ucla/remap"));
    nh.param("ndn/node_name", node_name, std::string("node0"));
    ncParams.prefix = ndn::NdnController::getInstancePrefix(prefix, node_name);
    nh.param("ndn/segment_length", ncParams.segmentLength, 2048);
    nh.param("ndn/daemon_host", ncParams.host, std::string("localhost"));
    nh.param("ndn/deamon_port", ncParams.port, 6363);

    ndn::NdnController ndnController(ncParams);

    if (0 == ndnController.startNdnProcessing())
    { 
      // setting up Active Tracks structure
      ndn::opt::ActiveTracks::Parameters atParams;
      atParams.jsonParameters = jsonParameters;

      ndn::opt::ActiveTracks activeTracks(atParams);

      // setting up Track Hint Publisher
      ndn::opt::TrackHintPublisher::Parameters thpParams;
      nh.param("hints/rate", thpParams.rate, 5);
      nh.param("hints/freshness_period", thpParams.freshnessPeriod, 200);
      thpParams.nh = nh;
      thpParams.ndnController = &ndnController;
      thpParams.activeTracks = &activeTracks;
      
      ndn::opt::TrackHintPublisher trackHintPublisher(thpParams);

      // setting up Track Publisher
      ndn::opt::TrackPublisher::Parameters  tpParams;
      nh.param("ndn/freshness_period", tpParams.freshnessPeriod, 5000);
      tpParams.bufferLength = 2048;
      tpParams.nh = ncParams.nh;
      tpParams.jsonParameters = jsonParameters;
      tpParams.activeTracks = &activeTracks;
      tpParams.ndnController = &ndnController; 

      ndn::opt::TrackPublisher trackPublisher(tpParams);

      trackPublisher.subscribe("input_topic");
      trackHintPublisher.start();
      
      ros::spin();

      trackHintPublisher.stop();
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
