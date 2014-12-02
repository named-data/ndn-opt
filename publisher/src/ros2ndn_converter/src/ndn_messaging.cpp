//
// ndn_messaging.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include "ndn_messaging.h"

using namespace ndn::opt;

NDNMessaging::NDNMessaging(Parameters& parameters):parameters_(parameters)
{

}

NDNMessaging::~NDNMessaging()
{

}

int 
NDNMessaging::startNdnProcessing()
{
	return 0;	
}

int 
NDNMessaging::stopNdnProcessing()
{
	return 0;
}

int
NDNMessaging::publishSequentialMessage(std::string& message)
{
	return 0;
}