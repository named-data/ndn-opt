// track_publisher.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include "track_publisher.h"

using namespace std;
using namespace ndn;
using namespace ndn::opt;

const std::string TrackPublisher::TracksNameComponent = "tracks";

TrackPublisher::TrackPublisher(const Parameters& parameters):
parameters_(parameters)
{
	ROS_DEBUG_STREAM("TrackPublisher ctor");
}

TrackPublisher::~TrackPublisher()
{
	unsubscribe();
	ROS_DEBUG_STREAM("TrackPublisher dtor");
}

int
TrackPublisher::subscribe(const string& topicName)
{
	ROS_DEBUG_STREAM("subscribing to topic " << topicName);
  	
  	trackingSub_ = parameters_.nh.subscribe<opt_msgs::TrackArray>(topicName, 1, &TrackPublisher::trackingCallback, this);	
  	
  	ROS_DEBUG_STREAM("subscription successful");

	return 0;
}

int
TrackPublisher::unsubscribe()
{
	ROS_DEBUG_STREAM("shutting down subscription...");

	trackingSub_.shutdown();

	ROS_DEBUG_STREAM("unsubscribed");
	return 0;
}

// private
void
TrackPublisher::trackingCallback(const opt_msgs::TrackArray::ConstPtr& tracking_msg)
{
	ROS_DEBUG_STREAM("callback trigerred. total tracks: " << tracking_msg->tracks.size());

	// iterate through tracks
  for (unsigned int i = 0; i < tracking_msg->tracks.size(); i++)
  {
    Jzon::Object current_track;
    current_track.Add("sec", int(tracking_msg->header.stamp.sec));
    current_track.Add("nsec", int(tracking_msg->header.stamp.nsec));
    current_track.Add("id", tracking_msg->tracks[i].id);
    current_track.Add("x", tracking_msg->tracks[i].x);
    current_track.Add("y", tracking_msg->tracks[i].y);
    current_track.Add("z", tracking_msg->tracks[i].height);

  	string json_string = parameters_.jsonParameters.toString(current_track);
    
    int seqNo = getNextSeqNum(tracking_msg->tracks[i].id);
    stringstream messageName;
    
    messageName << parameters_.basePrefix << "/" << TracksNameComponent << "/" << tracking_msg->tracks[i].id << "/" << seqNo;
    
    parameters_.ndnController->publishMessage(messageName.str(), parameters_.freshnessPeriod, (void*)json_string.c_str(), json_string.size());
    parameters_.activeTracks->newTrack(int(tracking_msg->header.stamp.sec), int(tracking_msg->header.stamp.nsec), tracking_msg->tracks[i].id, seqNo);

  	ROS_INFO_STREAM("published track " << tracking_msg->tracks[i].id << " with name " << messageName.str() << " json: " << json_string);
  }
}

int 
TrackPublisher::getNextSeqNum(const int trackId)
{
	map<int,int>::iterator it = trackSequenceNumbers_.find(trackId);

	if (it == trackSequenceNumbers_.end())
		trackSequenceNumbers_[trackId] = 0;
	else
		trackSequenceNumbers_[trackId] = it->second+1;

	return trackSequenceNumbers_[trackId];
}
