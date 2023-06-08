#ifndef OR_PROTOCOL_CONSTANTS_H_
#define OR_PROTOCOL_CONSTANTS_H_


namespace or_protocol {


// port used by the OR protocol
const int OR_PROTOCOL_PORT = 4568;

// minimum unit of delay for enforcing relay priority in nanoseconds
const int UNIT_DELAY = 10000000;

// maximum number of times a reliable packet should be re-transmitted before
// giving up
const int MAX_RETRY_COUNT = 2;

// each retransmission is delayed RETRY_DELAY_FACTOR ^ (attempt - 1)
const double RETRY_DELAY_FACTOR = 1.5;

// maximum duration to hold onto messages in a NodeState queue - at some point
// messages reach their intended destinations and aren't being relayed any
// longer and no longer need to be tracked
// NOTE 4 hop route * 4 candidate relays / hop * 2 factor of safety
const int MSG_BUFFER_DURATION = MAX_RETRY_COUNT * UNIT_DELAY * 16 * 2;

// thresholds for determining if a candidate relay node is "close" to the
// current node in terms of ETX
const double CLOSENESS_FACTOR = 2.0;
const double CLOSENESS_THRESHOLD = 4.0;

// threshold for determining of a candidate relay is close enough to the default
// path
const double NEAR_PATH_THRESHOLD = 2.0;

// the maximum allowable ETX between relays of the same transmitter
const double RELAY_ETX_THRESHOLD = 2.0;

// the time in milliseconds to wait before recomputing the routing table
const int ROUTING_UPDATE_INTERVAL = 1000;

// the beacon interval in milliseconds
const int BEACON_INTERVAL = 1000;

// jitter to inject into beacon interval in milliseconds
const int BEACON_JITTER = 100;

// number of beacon intervals to average over when estimating ETX
const unsigned int ETX_BUFFER_LEN = 8u;

// the time in seconds before a link is considered stale and set to ETX_MAX
const double ETX_STALE_TIME = 10.0;

// a large value for ETX used in cases where links have a delivery probability
// of zero
// TODO what are the implications of this? should it be lower?
const double ETX_MAX = 1e9;


}  // namespace or_protocol


#endif
