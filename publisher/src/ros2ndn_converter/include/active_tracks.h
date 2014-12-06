// active_tracks.h
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#ifndef __active_tracks_h__
#define __active_tracks_h__

#include <ros/ros.h>
#include <boost/thread.hpp>

#include "common.h"

namespace ndn
{
	namespace opt
	{
		/**
		 * This is a synchronization object used for storing active 
		 * tracks received from OpenPTrack. It stores any tracks added
		 * to it and ensures exclusive access to this data for all threads.
		 * Data is stored unless invalidated.
		 */
		class ActiveTracks
		{
		public:
			typedef struct _Parameters {
				JsonParameters jsonParameters;
			} Parameters;

			ActiveTracks(const Parameters& parameters);
			~ActiveTracks();

			void
			newTrack(const int sec, const int nsec, const int trackId, const int seqNo);

			void
			invalidate();

			bool
			isUpdated();

			std::string
			getCurrentHintData();

		private:
			Parameters parameters_;
			boost::mutex mutex_;
			bool isUpdated_;
			int latestTimestampSec_, latestTimestampNsec_;
			std::map<int, int> activeTracksSeqNumbers_;
		};
	}
}

#endif