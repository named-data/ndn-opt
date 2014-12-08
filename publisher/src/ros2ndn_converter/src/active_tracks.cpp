// active_tracks.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include "active_tracks.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;
using namespace ndn;
using namespace ndn::opt;

ActiveTracks::ActiveTracks(const Parameters& parameters):
parameters_(parameters),
isUpdated_(false),
latestTimestampSec_(0),
latestTimestampNsec_(0)
{
	ROS_DEBUG_STREAM("ActiveTracks ctor");
}

ActiveTracks::~ActiveTracks()
{
	ROS_DEBUG_STREAM("ActiveTracks dtor");
	invalidate();
}

void
ActiveTracks::newTrack(const int sec, const int nsec, const int trackId, const int seqNo)
{
	boost::mutex::scoped_lock scopedLock(mutex_);
	
	isUpdated_ = true;
	latestTimestampSec_ = sec;
	latestTimestampNsec_ = nsec;
	activeTracksSeqNumbers_[trackId] = seqNo;
}

void 
ActiveTracks::invalidate()
{
	boost::mutex::scoped_lock scopedLock(mutex_);
	isUpdated_ = false;
	latestTimestampSec_ = 0;
	latestTimestampNsec_ = 0;
	activeTracksSeqNumbers_.clear();
}

bool
ActiveTracks::isUpdated()
{
	boost::mutex::scoped_lock scopedLock(mutex_);	
	return isUpdated_;	
}

string
ActiveTracks::getCurrentHintData()
{
	boost::mutex::scoped_lock scopedLock(mutex_);
	Jzon::Object hintObject;

	hintObject.Add("sec", latestTimestampSec_);
	hintObject.Add("nsec", latestTimestampNsec_);

	Jzon::Array activeTracks;
	for (map<int,int>::iterator it = activeTracksSeqNumbers_.begin(); 
		it != activeTracksSeqNumbers_.end(); ++ it)
	{
		Jzon::Object activeTrack;
		activeTrack.Add("id", it->first);
		activeTrack.Add("seq", it->second);

		activeTracks.Add(activeTrack);
	}

	hintObject.Add("tracks", activeTracks);

	return parameters_.jsonParameters.toString(hintObject);
}

// private

