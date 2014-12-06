// common.h
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#ifndef __common_h__
#define __common_h__

#include "json.h"

typedef struct _JsonParameters {
	int jsonIndentSize;   // indent size for JSON message
	bool jsonNewline;      // use newlines (true) or not (false) in JSON messages
	bool jsonSpacing;      // use spacing (true) or not (false) in JSON messages
	bool jsonUseTabs;     // use tabs (true) or not (false) in JSON messages

	std::string toString(const Jzon::Object& jsonObject);
} JsonParameters;

#endif