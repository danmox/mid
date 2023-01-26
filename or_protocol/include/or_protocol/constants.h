#ifndef OR_PROTOCOL_CONSTANTS_H_
#define OR_PROTOCOL_CONSTANTS_H_


namespace or_protocol {


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

}  // namespace or_protocol


#endif
