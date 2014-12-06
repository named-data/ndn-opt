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

using namespace ndn;
using namespace std;

const string NdnController::NdnAppComponent = "opt";

NdnController::NdnController(const Parameters& parameters):
parameters_(parameters),
keyChain_(new KeyChain())
{
	ROS_DEBUG_STREAM("NdnController ctor");

	// create face
	ROS_DEBUG_STREAM("creating face...");
	face_.reset(new Face(parameters_.host.c_str(), parameters_.port));
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
		this, _1));

	ROS_DEBUG_STREAM("prefix registration intiated. starting process events loop...");

	// start processEvents loop
	startProcessEventsLoop();

	ROS_DEBUG_STREAM("NDN processing started");

	return 0;	
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
	}

	ROS_DEBUG_STREAM("published data " << name);

	return 0;
}

string
NdnController::getInstancePrefix(const string& hubPrefix, const string& nodeName)
{
	ros::Time time = ros::Time::now();
	stringstream ss;
	ss << hubPrefix << "/" << NdnAppComponent << "/" << nodeName << "/" << time.sec;

	return ss.str();
}

// private
void
NdnController::onRegisterFailed(const ptr_lib::shared_ptr<const Name>& prefix)
{
	ROS_DEBUG_STREAM("prefix registration failed");

	throw new std::runtime_error("prefix registration failed");
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

	face_->processEvents();
}
