#ifndef OR_PROTOCOL_NODE_STATE_H_
#define OR_PROTOCOL_NODE_STATE_H_


#include <cstdint>
#include <deque>
#include <unordered_map>

#include <or_protocol_msgs/Header.h>


namespace or_protocol {


class NodeState
{
  public:

    // update the received message queue used for determining if a message has:
    // 1) already been received and doesn't need to be processed again or 2) has
    // already been relayed by a higher priority node; returns if the message is
    // new
    bool update_queue(const or_protocol_msgs::Header& header);

    // TODO compute transmission statistics
    void update_stats(const or_protocol_msgs::Header& header);

    // return the priority of a message (assumes a message with the same source
    // and sequence number has already been added to the queue)
    int priority(const int seq);

  private:

    // the history of messages received kept as a deque for quickly determining
    // the oldest message in the queue and as a map for quickly determining if a
    // message has been received before and the highest received priority; these
    // two data structures are kept synchronized
    std::unordered_map<uint32_t,unsigned int> msg_hist_map;
    std::deque<uint32_t> msg_hist_deque;

    // maximum number of messages to keep in the queue - at some point messages
    // reach their intended destinations and aren't being relayed any longer and
    // don't need to be tracked anymore
    // TODO also remove messages that have been acknowledged
    const size_t buff_capacity = 200;
};


}  // namespace or_protocol


#endif
