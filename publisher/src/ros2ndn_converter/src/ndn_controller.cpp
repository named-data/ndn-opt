// ndn_controller.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "ndn_controller.h"
#include "common.h"

using namespace ndn;
using namespace std;

static int instanceStartTime_ = 0;

NdnController::NdnController(const Parameters& parameters):
parameters_(parameters),
keyChain_(new KeyChain())
{
	ROS_DEBUG_STREAM("NdnController ctor");

	// create face: zhehao: use UnixTransport for local host names
	ROS_DEBUG_STREAM("creating face...");
	if (parameters_.host != "localhost" && parameters_.host != "127.0.0.1") {
		face_.reset(new Face(parameters_.host.c_str(), parameters_.port));
	} else {
		face_.reset(new Face());
	}

	face_->setCommandSigningInfo(*keyChain_, keyChain_->getDefaultCertificateName());

	ROS_DEBUG_STREAM("face created. creating memory cache...");

	// create memory content cache
	memoryCache_.reset(new MemoryContentCache(face_.get()));

	ROS_DEBUG_STREAM("memory cache creted");	
}

NdnController::~NdnController()
{
	stopNdnProcessing();
	ROS_DEBUG_STREAM("NdnController dtor");
}

int 
NdnController::startNdnProcessing()
{
	ROS_DEBUG_STREAM("start NDN processing...");
	ROS_DEBUG_STREAM("registering prefix " << parameters_.prefix);

	// register prefix
	Name instancePrefix(parameters_.prefix);
	memoryCache_->registerPrefix(instancePrefix, bind(&NdnController::onRegisterFailed,
		this, _1), bind(&NdnController::onInterest, this, _1, _2, _3, _4));

	ROS_DEBUG_STREAM("prefix registration intiated. starting process events loop...");

	// start processEvents loop
	startProcessEventsLoop();

	ROS_DEBUG_STREAM("NDN processing started");

	return 0;	
}

// Zhehao: onInterest implementation
// Note: Ignoring if the same interest should be handled here.
void
NdnController::onInterest
  (const ptr_lib::shared_ptr<const Name>& prefix,
   const ptr_lib::shared_ptr<const Interest>& interest, Transport& transport,
   uint64_t registerPrefixId)
{
	ROS_DEBUG_STREAM("PendingInterest added");
	pendingInterestTable_.push_back(ptr_lib::shared_ptr<PendingInterest>
	  (new PendingInterest(interest, transport)));
}

// Zhehao: pending interest class method implementation
PendingInterest::PendingInterest
  (const ptr_lib::shared_ptr<const Interest>& interest, Transport& transport)
  : interest_(interest), transport_(transport)
{
  // Set up timeoutTime_.
  if (interest_->getInterestLifetimeMilliseconds() >= 0.0)
    timeoutTimeMilliseconds_ = ndn_getNowMilliseconds() +
      interest_->getInterestLifetimeMilliseconds();
  else
    // No timeout.
    timeoutTimeMilliseconds_ = -1.0;
}

// Zhehao: Copied methods from Jeff's c/util/time.h, which is not exposed
ndn_MillisecondsSince1970
ndn::ndn_getNowMilliseconds()
{
	struct timeval t;
	gettimeofday(&t, 0);
	return t.tv_sec * 1000.0 + t.tv_usec / 1000.0;
}

int 
NdnController::stopNdnProcessing()
{
	ROS_DEBUG_STREAM("stop NDN processing...");
	ROS_DEBUG_STREAM("stoping processing events...");

	faceEventsTimer_.stop();
	
	ROS_DEBUG_STREAM("processing events stopped. unregistering prefixes...");
	memoryCache_->unregisterAll();

	ROS_DEBUG_STREAM("prefixes unregistered.");
	ROS_DEBUG_STREAM("NDN processing stopped");
	return 0;
}

int
NdnController::publishMessage(const string& name, const int& dataFreshnessMs, const void* message, const int& messageLength)
{
	Name dataName(name);
	Data ndnData(dataName);

	ndnData.getMetaInfo().setFreshnessPeriod(dataFreshnessMs);

	int ndnDataLength = (messageLength > parameters_.segmentLength)? parameters_.segmentLength : messageLength;
	
	ndnData.setContent((const uint8_t*)message, ndnDataLength);
	keyChain_->sign(ndnData, keyChain_->getDefaultCertificateName());

	{
		boost::mutex::scoped_lock scopedLock(faceMutex_);
		memoryCache_->add(ndnData);

		// Zhehao: add pendingInterestTable operations after adding to memory contentcache		
		// Remove timed-out interests and check if the data packet matches any pending
		// interest.
		// Go backwards through the list so we can erase entries.
		MillisecondsSince1970 nowMilliseconds = ndn_getNowMilliseconds();
		for (int i = (int)pendingInterestTable_.size() - 1; i >= 0; --i) {
			if (pendingInterestTable_[i]->isTimedOut(nowMilliseconds)) {
				pendingInterestTable_.erase(pendingInterestTable_.begin() + i);
				continue;
			}

			if (pendingInterestTable_[i]->getInterest()->matchesName(ndnData.getName())) {
				try {
					// Send to the same transport from the original call to onInterest.
					// wireEncode returns the cached encoding if available.
					pendingInterestTable_[i]->getTransport().send(*ndnData.wireEncode());
				}
				catch (std::exception& e) {
					ROS_ERROR_STREAM("got exception: " << e.what());
				}

				// The pending interest is satisfied, so remove it.
				pendingInterestTable_.erase(pendingInterestTable_.begin() + i);
			}
		}   
	}

	ROS_DEBUG_STREAM("published data " << name);

	return 0;
}

string
NdnController::getBasePrefix()
{
	stringstream ss;
	ss << parameters_.prefix << "/" << getInstanceStartTime();

	return ss.str();
}

string
NdnController::getInstancePrefix(const string& hubPrefix, const string& nodeName)
{	
	stringstream ss;
	ss << hubPrefix << "/" << NameComponents::NameComponentApp << "/" << nodeName;

	return ss.str();
}

void 
NdnController::instanceStart()
{
	if (instanceStartTime_ == 0)
	{
		ros::Time time = ros::Time::now();
		instanceStartTime_ = time.sec;	
	}
}

int
NdnController::getInstanceStartTime()
{
	return instanceStartTime_;
}

// private
void
NdnController::onRegisterFailed(const ptr_lib::shared_ptr<const Name>& prefix)
{
	ROS_DEBUG_STREAM("prefix registration failed");

	throw new std::runtime_error("prefix registration failed");
}

// Zhehao: create exception does not yet work as expected...testing
void
NdnController::createException(const ros::WallTimerEvent& timerEvent)
{
	//std::cout << *param << std::endl;
	// Time not actually used...
	std::cout << "Manual exception" << endl;
	FILE *fp;
	fp = fopen("testfile", "w");
	fputs("bal", fp);
	fclose(fp);
	std::string("abc").substr(10);	
	//throw new std::runtime_error("Manual error for test.");
}

// Zhehao: add manual exceptions for test
void
NdnController::scheduleException(int time)
{
	//parameters_.nh.createTimer(ros::Duration(time), createException, true);
	fancyEventsTimer_ = parameters_.nh.createWallTimer(ros::WallDuration(time), &NdnController::createException, this);	
	std::cout << "Exception scheduled." << endl;
}

void
NdnController::startProcessEventsLoop()
{
	ROS_DEBUG_STREAM("starting face events timer...");	

	faceEventsTimer_ = parameters_.nh.createWallTimer(ros::WallDuration(0.001), &NdnController::processFaceEventsCallback, this);

	ROS_DEBUG_STREAM("timer started");
}

void 
NdnController::processFaceEventsCallback(const ros::WallTimerEvent& timerEvent)
{
	boost::mutex::scoped_lock scopedLock(faceMutex_);
	//std::cout << "anything" << std::endl;
	face_->processEvents();
}
