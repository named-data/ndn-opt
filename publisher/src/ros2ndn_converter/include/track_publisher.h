// track_publisher.h
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#ifndef __track_publisher_h__
#define __track_publisher_h__

#include <ros/ros.h>
#include <opt_msgs/TrackArray.h>
#include <json.h>

#include "common.h"
#include "ndn_controller.h"
#include "active_tracks.h"

namespace ndn
{
	namespace opt 
	{
		/**
		 * This class performs receiving track samples from OpenPTrack, 
		 * extracting relevant information from received data and passes 
		 * extracted data in the form of Json to NdnController in order
		 * to publish this data in NDN network.
		 */
		class TrackPublisher
		{
		public:
			static const std::string TracksNameComponent;

			typedef struct _Parameters {
				ros::NodeHandle nh;
				int bufferLength;
				int freshnessPeriod;
				std::string basePrefix;
				JsonParameters jsonParameters;

				ActiveTracks* activeTracks;
				NdnController* ndnController;
			} Parameters; 

			TrackPublisher(const Parameters& parameters);
			~TrackPublisher();

			int
			subscribe(const std::string& topicName);

			int 
			unsubscribe();

		private:
			Parameters parameters_;
			ros::Subscriber trackingSub_;
			std::map<int, int> trackSequenceNumbers_;

			void 
			trackingCallback(const opt_msgs::TrackArray::ConstPtr& tracking_msg);

			int 
			getNextSeqNum(const int trackId);
		};
	}
}

#endif
