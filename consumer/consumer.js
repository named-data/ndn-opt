// consumer.js
//
//  Copyright 2014 Regents of the University of California
//  For licensing details see the LICENSE file.
//
//  Author:  Zhehao Wang

/**
 * Consumer creates a consumer for ndn-opt; The consumer follows the protocol
 * described here: https://github.com/named-data/ndn-opt/tree/master/publisher
 * Multithread access of consumer is not taken care of in this implementation.
 *
 * @param {Face} The websocket face which consumer uses;
 * @param {Name} The [root] prefix that producer uses; Before [space_name] component;
 * @param {Name} The [space_name] prefix that producer uses; Full prefix = [root]/[space_name]
 * @param {function(decoded JSON object)} Function called whenever the consumer fetched track data
 */
var Consumer = function(face, root, spaceName, displayCallback)
{
  this.face = face;
  this.prefix = new Name(root);
  this.prefix.append(spaceName);
  
  this.startTimeComponent = new Name.Component();
  // this records the indices of the tracks that have been active since the start of this run.
  this.activeTracks = [];
  
  // All fetched track data is stored in this array: may want to organize it.
  // Are we expecting a large number of entries, which means I should flush this array at some point?
  this.trackData = [];
  
  // Display callback is called once track data is received
  this.displayCallback = displayCallback;
};

Consumer.prototype.getTrackData = function()
{
  return this.trackData;
};

Consumer.prototype.getActiveTrack = function()
{
  return this.activeTracks;
};

// Expected data name: [root]/opt/[node_num]/[start_timestamp]/tracks/[track_num]/[seq_num]
Consumer.prototype.onTrackData = function(interest, data)
{
  console.log(data.getName().toUri());
  var parsedTrack = JSON.parse(data.getContent().buf());
  this.trackData.push(parsedTrack);
  
  if (this.displayCallback) {
    this.displayCallback(parsedTrack);
  }
  
  var trackId = parseInt(interest.getName().get
    (ProducerNameComponents.trackIdOffset).toEscapedString());
  var activeTrackIndex = this.indexOfTrackId(trackId);
  if (activeTrackIndex != -1) {
    this.activeTracks[activeTrackIndex].timeoutCnt = 0;
  }
  else {
	this.activeTracks.push({"id": trackId,
						   "timeoutCnt": 0});
  }
  
  var receivedSeq = interest.getName().get(-1).toEscapedString();
  var seq = parseInt(receivedSeq) + 1;
  
  var trackInterest = new Interest(data.getName().getPrefix(-1).append(seq.toString()));
  
  console.log(trackInterest.getName().toUri());
  
  trackInterest.setMustBeFresh(true);
  trackInterest.setInterestLifetimeMilliseconds(Config.defaultTrackLifetime);
  this.face.expressInterest
    (trackInterest, this.onTrackData.bind(this), this.onTrackTimeout.bind(this));
};

Consumer.prototype.onTrackTimeout = function(interest)
{
  console.log("onTrackTimeout called: " + interest.getName().toUri());
  console.log("Host: " + this.face.connectionInfo.toString());
  
  // Express timeout interest; this may not be needed for track fetching:
  // we can reexpress the instant timeout happens.
  //var timeout = new Interest(new Name("/local/timeout"));
  //timeout.setInterestLifetimeMilliseconds(Config.defaultReexpressInterval);
  
  // trackId is always assumed to be 
  var trackId = parseInt(interest.getName().get
    (ProducerNameComponents.trackIdOffset).toEscapedString());
  var activeTrackIndex = this.indexOfTrackId(trackId);
  
  if (activeTrackIndex != -1) {
    if (this.activeTracks[activeTrackIndex].timeoutCnt < Config.trackTimeoutThreshold) {
	  this.face.expressInterest
	    (interest, this.onTrackData.bind(this), this.onTrackTimeout.bind(this));
	  this.activeTracks[activeTrackIndex].timeoutCnt ++;
	}
	else {
	  this.activeTracks.splice(activeTrackIndex, 1);
	}
  }
};

Consumer.prototype.indexOfTrackId = function(id)
{
  for (var i = 0; i < this.activeTracks.length; i++) {
    if (this.activeTracks[i].id == id) {
      return i;
    }
  }
  return -1;
}

// Expected data name: [root]/opt/[node_num]/[start_timestamp]/track_hint/[num]
Consumer.prototype.onHintData = function(interest, data)
{
  console.log("onHintData called: " + data.getName().toUri());
  var parsedHint = JSON.parse(data.getContent().buf());
  
  for (var i = 0; i < parsedHint.tracks.length; i++) {
    
    // The consumer ignores the sequence number field in the hint for now;
    // As the consumer assumes it's getting the latest via outstanding interest.
    // Right now the consumer does not stop fetching tracks that have become inactive.
    if (this.indexOfTrackId(parsedHint.tracks[i].id) == -1) {
      this.fetchTrack(parsedHint.tracks[i].id);
      this.activeTracks.push({"id": parsedHint.tracks[i].id,
                             "timeoutCnt": 0});
    }
  }
  // express interest for the new hint
  var hintName = new Name(this.prefix);
  hintName.append(this.startTimeComponent).append(ProducerNameComponents.trackHint);
  
  var hintInterest = new Interest(hintName);
  
  // should exclude the older one
  var exclude = new Exclude();
  
  exclude.appendAny();
  exclude.appendComponent(data.getName().get(-1));
  hintInterest.setExclude(exclude);
  
  hintInterest.setMustBeFresh(true);
  hintInterest.setInterestLifetimeMilliseconds(Config.defaultHintLifetime);
  // for hint interest, the rightMostChild is preferred.
  hintInterest.setChildSelector(1);
  
  this.face.expressInterest
    (hintInterest, this.onHintData.bind(this), this.onHintTimeout.bind(this));
};

Consumer.prototype.onHintTimeout = function(interest)
{
  console.log("onHintTimeout called: " + interest.getName().toUri());
  console.log("Host: " + this.face.connectionInfo.toString());
  
  // Express timeout interest; this may not be needed for track fetching:
  // we can express the instant timeout happens.
  //var timeout = new Interest(new Name("/local/timeout"));
  //timeout.setInterestLifetimeMilliseconds(Config.defaultReexpressInterval);
  
  this.face.expressInterest
    (interest, this.onHintData.bind(this), this.onHintTimeout.bind(this));
};

// Meta data not yet published
Consumer.prototype.onMetaData = function(interest, data)
{
  
};

Consumer.prototype.onMetaTimeout = function(interest)
{
  console.log("onTimeout called for interest " + interest.getName().toUri());
  console.log("Host: " + this.face.connectionInfo.toString());
};

Consumer.prototype.onInitialData = function(interest, data)
{
  var dataName = data.getName();
  console.log("Initial data received: " + dataName.toUri());
  
  // Data name should contain at least [full prefix] + [start_timestamp] + [tracks/track_hint]
  if (dataName.size() > this.prefix.size() + 1) {
    this.startTimeComponent = dataName.get(this.prefix.size());
    this.fetchTrackHint();
  }
  else {
    console.log("Initial interest fetched unexpected data: " + dataName.toUri());
  }
};

Consumer.prototype.onInitialTimeout = function(interest)
{
  console.log("Initial interest times out: " + interest.getName().toUri());
  
  // Express timeout interest 
  var timeout = new Interest(new Name("/local/timeout"));
  timeout.setInterestLifetimeMilliseconds(Config.initialReexpressInterval);
  
  this.face.expressInterest
    (timeout, this.dummyOnData, this.reexpressInitialInterest.bind(this));
};

Consumer.prototype.reexpressInitialInterest = function(interest)
{
  var initialInterest = new Interest(this.prefix);
  initialInterest.setMustBeFresh(true);
  initialInterest.setInterestLifetimeMilliseconds(Config.defaultInitialLifetime);
  // for initial interest, the rightMostChild is preferred
  initialInterest.setChildSelector(1);
  
  this.face.expressInterest
    (initialInterest, this.onInitialData.bind(this), this.onInitialTimeout.bind(this));
};

Consumer.prototype.dummyOnData = function(interest, data)
{
  // don't expect this to get called
  console.log("DummyOnData called.");
}

// Start fetching the track from sequence number 0
Consumer.prototype.fetchTrack = function(trackId)
{
  var trackName = new Name(this.prefix);
  
  trackName.append
    (this.startTimeComponent).append
    (ProducerNameComponents.tracks).append
    (trackId.toString()).append("0");
  
  var trackInterest = new Interest(trackName);
  trackInterest.setMustBeFresh(true);
  this.face.expressInterest
    (trackInterest, this.onTrackData.bind(this), this.onTrackTimeout.bind(this));
};

Consumer.prototype.fetchTrackHint = function()
{
  var hintName = new Name(this.prefix);
  hintName.append(this.startTimeComponent).append(ProducerNameComponents.trackHint);
  
  var hintInterest = new Interest(hintName);
  hintInterest.setMustBeFresh(true);
  hintInterest.setInterestLifetimeMilliseconds(Config.defaultHintLifetime)
  this.face.expressInterest
    (hintInterest, this.onHintData.bind(this), this.onHintTimeout.bind(this));
};

/**
 * Consumer starts by issuing interest with given [root prefix]/[space name], 
 * and try to fetch the right most piece of matching data, and extract its timestamp.
 */
Consumer.prototype.start = function() {
  // fetch initial data, since consumer does not know about the value of starting time component
  var initialInterest = new Interest(this.prefix);
  initialInterest.setMustBeFresh(true);
  initialInterest.setInterestLifetimeMilliseconds(Config.defaultInitialLifetime);
  // for initial interest, the rightMostChild is preferred
  initialInterest.setChildSelector(1);
  
  this.face.expressInterest
    (initialInterest, this.onInitialData.bind(this), this.onInitialTimeout.bind(this));
};
