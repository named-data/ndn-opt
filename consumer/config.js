// config.js
//
//  Copyright 2014 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Zhehao Wang

// Connection configuration, as connectivity to default WS on NDN testbed is not guaranteed

Config = {
  // Face configuration
  hostName	: "localhost",
  wsPort	: 19696,
  
  // Interest configuration
  initialReexpressInterval	: 1000,
  defaultInitialLifetime	: 2000,
  defaultTrackLifetime		: 250,
  defaultHintLifetime		: 500,
  
  // The threshold for number of timeouts received in a row to decide not to fetch certain
  // track anymore
  trackTimeoutThreshold		: 20
};

ProducerNameComponents = {
  tracks		: "tracks",
  trackHint		: "track_hint",
  trackIdOffset	: -2
};