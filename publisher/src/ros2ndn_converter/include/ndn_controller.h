// ndn_controller.h
//
//	Copyright 2013 Regents of the University of California
//	For licensing details see the LICENSE file.
//
//	Author:	Peter Gusev
//

#ifndef __ndn_controller_h__
#define __ndn_controller_h__

#include <vector>
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

#include <time.h>
#include <ndn-cpp/c/common.h>

 namespace ndn 
 {
	ndn_MillisecondsSince1970
	ndn_getNowMilliseconds();

	/**
	 * A PendingInterest holds an interest which onInterest received but could
	 * not satisfy. When we add a new data packet to the contentCache_, we will
	 * also check if it satisfies a pending interest.
	 */
	class PendingInterest {
	public:
		/**
		 * Create a new PendingInterest and set the timeoutTime_ based on the current time and the interest lifetime.
		 * @param interest A shared_ptr for the interest.
		 * @param transport The transport from the onInterest callback. If the
		 * interest is satisfied later by a new data packet, we will send the data
		 * packet to the transport.
		 */
		PendingInterest
			(const ptr_lib::shared_ptr<const Interest>& interest,
			 Face& transport);

		/**
		 * Return the interest given to the constructor.
		 */
		const ptr_lib::shared_ptr<const Interest>&
		getInterest() { return interest_; }

		/**
		 * Return the transport given to the constructor.
                 */
		Face&
		getFace() { return transport_; }

		/**
		 * Check if this interest is timed out.
		 * @param nowMilliseconds The current time in milliseconds from ndn_getNowMilliseconds.
		 * @return true if this interest timed out, otherwise false.
		 */
		bool
		isTimedOut(MillisecondsSince1970 nowMilliseconds)
		{
			return timeoutTimeMilliseconds_ >= 0.0 && nowMilliseconds >= timeoutTimeMilliseconds_;
		}

	private:
		ptr_lib::shared_ptr<const Interest> interest_;
		Face& transport_;
		MillisecondsSince1970 timeoutTimeMilliseconds_; 
		/**< The time when the
		 * interest times out in milliseconds according to ndn_getNowMilliseconds,
		 * or -1 for no timeout. */
	};

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
		
		// Zhehao: onInterest(onDataNotFound) method stores the interest into an application level PIT
		void
		onInterestCallback
			(const ndn::ptr_lib::shared_ptr<const ndn::Name>& prefix,
   const ndn::ptr_lib::shared_ptr<const ndn::Interest>& interest, ndn::Face& face,
   uint64_t registeredPrefixId, const ndn::ptr_lib::shared_ptr<const ndn::InterestFilter>& filter);

		int
		publishMessage(const std::string& name, const int& dataFreshnessMs, const void* message, const int& messageLength);

		std::string
		getBasePrefix();

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
		std::vector<ptr_lib::shared_ptr<PendingInterest> > pendingInterestTable_;

		void 
		startProcessEventsLoop();

		void
		processFaceEventsCallback(const ros::WallTimerEvent& timerEvent);

		void
		onRegisterFailed(const ptr_lib::shared_ptr<const Name>& prefix);
 	};
 }

 #endif
