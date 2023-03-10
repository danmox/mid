#ifndef OR_PROTOCOL_CONSTANTS_H_
#define OR_PROTOCOL_CONSTANTS_H_


namespace or_protocol {


// port used by the OR protocol
#define OR_PROTOCOL_PORT 4568

// minimum unit of delay for enforcing relay priority in nanoseconds
#define UNIT_DELAY 10000000

// duration of time in nanoseconds to wait for an ACK before re-transmitting
// a reliable packet, taking into account the worst case forwarding path for
// the packet and ACK
#define RETRY_DELAY 20 * UNIT_DELAY

// maximum number of times a reliable packet should be re-transmitted before
// giving up
#define MAX_RETRY_COUNT 2

// maximum number of messages to keep in a NodeState queue - at some point
// messages reach their intended destinations and aren't being relayed any
// longer and no longer need to be tracked
#define MSG_BUFFER_CAPACITY 200

// thresholds for determining if a candidate relay node is "close" to the
// current node in ETX
#define CLOSENESS_FACTOR 2.0
#define CLOSENESS_THRESHOLD 4.0

// threshold for determining of a candidate relay is close enough to the default
// path
#define NEAR_PATH_THRESHOLD 2.0

// the maximum number of relays to use at each hop
#define MAX_RELAY_COUNT 4

// the maximum allowable ETX between relays of the same transmitter
#define RELAY_ETX_THRESHOLD 2.0


}  // namespace or_protocol


#endif
