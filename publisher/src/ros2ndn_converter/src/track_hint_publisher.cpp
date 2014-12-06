// track_hint_publisher.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include "track_hint_publisher.h"

using namespace std;
using namespace ndn;
using namespace ndn::opt;

TrackHintPublisher::TrackHintPublisher(const Parameters& parameters):
parameters_(parameters)
{
	ROS_DEBUG_STREAM("TrackHintPublisher ctor");
}

TrackHintPublisher::~TrackHintPublisher()
{
	ROS_DEBUG_STREAM("TrackHintPublisher dtor");
}

void
TrackHintPublisher::start()
{
	ROS_DEBUG_STREAM("starting TrackHintPublisher...");
	sampleTimer_ = parameters_.nh.createWallTimer(ros::WallDuration(1./float(parameters_.rate), &TrackHintPublisher::publishTrackHint, this));
	ROS_DEBUG_STREAM("TrackHintPublisher started.");
}

void
TrackHintPublisher::stop()
{
	sampleTimer_.stop();
}

// private
void
TrackHintPublisher::publishTrackHint(const ros::WallTimerEvent& timerEvent)
{
	if (parameters_.activeTracks->isUpdated())
	{
		ros::Time time = ros::Time::now();
		int instanceStartTimeSec = NdnController::getInstanceStartTime();
		int offsetMs = int((time.sec - instanceStartTimeSec)*1000 + double(time.nsec)/1000000.);
		string jsonString = parameters_.activeTracks->getCurrentHintData();
		stringstream hintName;

		hintName << parameters_.ndnController->getBasePrefix() << NameComponents::NameComponentTrackHint << offsetMs;
		parameters_.ndnController->publishMessage(hintName.str(), parameters_.freshnessPeriod, 
			(const void*)jsonString.c_str(), jsonString.size());
	}
}
