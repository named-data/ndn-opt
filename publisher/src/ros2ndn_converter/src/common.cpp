// common.cpp
//
//  Copyright 2013 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Peter Gusev
//

#include "common.h"

using namespace std;

const std::string NameComponents::NameComponentApp = "opt";
const std::string NameComponents::NameComponentTracks = "tracks";
const std::string NameComponents::NameComponentTrackHints = "track_hint";
const std::string NameComponents::NameComponentMeta = "_meta";

string
JsonParameters::toString(const Jzon::Object& jsonObject)
{
  	Jzon::Format message_format = Jzon::StandardFormat;
  	message_format.indentSize = jsonIndentSize;
  	message_format.newline = jsonNewline;
  	message_format.spacing = jsonSpacing;
  	message_format.useTabs = jsonUseTabs;

  	Jzon::Writer writer(jsonObject, message_format);
  	writer.Write();

  	return writer.GetResult();	
}
