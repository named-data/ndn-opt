# OpenPTrack NDN consumer in browser

### Prerequisites 
1. [NDN-JS](https://github.com/named-data/ndn-js/blob/master/build/ndn.js)

### How to use
1. Clone repository
2. Replace the link to ndn.js in consumer directory with the actual ndn.js, which can be grabbed from the repository in the link above.
3. Edit [config.js](https://github.com/named-data/ndn-opt/blob/master/consumer/config.js) for namespace and face setup
4. Launch [consumer.html](https://github.com/named-data/ndn-opt/blob/master/consumer/consumer.html) in a browser, and click "Start consumer" button.
5. Make sure producer's running, and the nfd websocket you are connecting to has a route to the producer

### Consumer
The consumer tries to fetch the [starting timestamp] with configured prefix, and fetches track hint and data under that [starting timestamp]

The consumer maintains an outstanding interest for track hint, with older timestamps excluded. 

For each new active track mentioned in track hint, the consumer maintains an outstanding interest for the new data sequence number.

The consumer stops maintaining outstanding interest for a track when a series of timeouts is received.