// track_hint_publisher.h
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#ifndef __track_hint_publisher_h__
#define __track_hint_publisher_h__

#include "active_tracks.h"
#include "ndn_controller.h"

namespace ndn
{
	namespace opt
	{
		/**
		 * This class queries ActiveTracks instance periodically and, if 
		 * there was an update, publishes hint data on NDN network.
		 */
		class TrackHintPublisher
		{
		public:
			typedef struct _Parameters {
				int rate;
				int freshnessPeriod;
				
				ros::NodeHandle nh;
				ActiveTracks* activeTracks;
				NdnController* ndnController;
			} Parameters;

			TrackHintPublisher(const Parameters& parameters);
			~TrackHintPublisher();

			void
			start();

			void
			stop();

		private:
			Parameters parameters_;
			ros::WallTimer sampleTimer_;

			void
			publishTrackHint(const ros::WallTimerEvent& timerEvent);
		};
	}
}

#endif