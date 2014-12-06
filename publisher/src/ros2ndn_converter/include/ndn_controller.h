// ndn_controller.h
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#ifndef __ndn_controller_h__
#define __ndn_controller_h__

#include <ros/ros.h>
#include <boost/thread.hpp>

#include <ndn-cpp/common.hpp>
#include <ndn-cpp/interest.hpp>
#include <ndn-cpp/data.hpp>
#include <ndn-cpp/transport/tcp-transport.hpp>
#include <ndn-cpp/transport/udp-transport.hpp>
#include <ndn-cpp/face.hpp>
#include <ndn-cpp/util/memory-content-cache.hpp>
#include <ndn-cpp/security/key-chain.hpp>

 namespace ndn 
 {
 	/**
 	* This class provides support for publishing on NDN network:
 	* 	- publishing data using Memory Content Cache
 	* 	- signing data
 	* Each call returns 0 on success and negative value which
 	* represents error code on failures.
 	* 
 	* NOTE: there is a segmentLenght parameter which defines the 
 	* maximum size of NDN segment to be published. If data size is
 	* greater than this size, data is truncated.
 	*/
 	class NdnController {
	public:
		typedef struct _Parameters
		{
			ros::NodeHandle nh;
			std::string host; 	// NDN daemon host name
			int port; 			// NDN daemon port number
			std::string prefix; // prefix to be registered
			int segmentLength;	// NDN segment size
		} Parameters;

		NdnController(const Parameters& parameters);
		~NdnController();

		int 
		startNdnProcessing();

		int 
		stopNdnProcessing();

		int
		publishMessage(const std::string& name, const int& dataFreshnessMs, const void* message, const int& messageLength);

		std::string
		getBasePrefix()
		{ return parameters_.prefix; }

		static std::string
		getInstancePrefix(const std::string& hubPrefix, const std::string& nodeName);

		static int 
		getInstanceStartTime();

		static void
		instanceStart();

	private:
		ros::WallTimer faceEventsTimer_;
		boost::mutex faceMutex_;
		Parameters parameters_;
		ptr_lib::shared_ptr<ndn::KeyChain> keyChain_;
		ptr_lib::shared_ptr<ndn::Face> face_;
		ptr_lib::shared_ptr<MemoryContentCache> memoryCache_;

		void 
		startProcessEventsLoop();

		void
		processFaceEventsCallback(const ros::WallTimerEvent& timerEvent);

		void
		onRegisterFailed(const ptr_lib::shared_ptr<const Name>& prefix);
 	};
 }

 #endif
