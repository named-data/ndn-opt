// 
// ndn_messaging.h
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#ifndef __ndn_messaging_h__
#define __ndn_messaging_h__

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
	namespace opt 
	{
		class NDNMessaging
		{
		public:
			typedef struct _Parameters
			{
				std::string prefix, nodeName;
				int segmentLength;
				int dataFreshnessMs;
			} Parameters;

			NDNMessaging(Parameters& parameters);
			~NDNMessaging();

			int 
			startNdnProcessing();

			int 
			stopNdnProcessing();

			int
			publishSequentialMessage(std::string& message);

		private:
			Parameters parameters_;
			ptr_lib::shared_ptr<ndn::KeyChain> keyChain_;
			ptr_lib::shared_ptr<ndn::Face> face_;
			ptr_lib::shared_ptr<MemoryContentCache> memoryCache_;
		};
	} // opt
} // ndn

#endif
