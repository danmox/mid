#ifndef OR_PROTOCOL_NODE_STATE_H_
#define OR_PROTOCOL_NODE_STATE_H_


#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include <or_protocol/constants.h>

#include <or_protocol_msgs/Header.h>


namespace or_protocol {


struct AttemptPriorityPair
{
    unsigned int attempt;
    unsigned int priority;
};


struct SeqAttemptPair
{
    uint32_t seq;
    unsigned int attempt;
};


struct MsgStatus
{
    bool is_new_msg;
    bool is_new_seq;
};


class NodeState
{
  public:
    // update the received message queue used for determining if a message has:
    // 1) already been received and doesn't need to be processed again or 2) has
    // already been relayed by a higher priority node; returns if the message is
    // new
    MsgStatus update_queue(const or_protocol_msgs::Header& header);

    // updated the received message queue when an ACK has been received (i.e.
    // zero out the priority simulating the reception of a relay a higher
    // priority node)
    void ack_msg(const uint32_t seq);

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
    std::unordered_map<uint32_t, AttemptPriorityPair> msg_hist_map;
    std::deque<SeqAttemptPair> msg_hist_deque;
};


typedef std::unordered_map<int, std::unordered_map<int, double>> ETXMap;
typedef std::unordered_map<int, std::vector<int>> IntVectorMap;
typedef or_protocol_msgs::Header::_relays_type RelayArray;
typedef std::unordered_map<int, RelayArray> IntArrayMap;
typedef std::unordered_map<int, IntVectorMap> VariableRoutingMap;
typedef std::unordered_map<int, IntArrayMap> FixedRoutingMap;


const int MAX_RELAY_COUNT = RelayArray::max_size();


class NetworkState
{
  public:
    // update the received message queue for the given node
    MsgStatus update_queue(const int node_id, const or_protocol_msgs::Header& header);

    // update the received message queue when an ACK has been received
    void ack_msg(const int dest_id, const uint32_t seq);

    // returns the highest priority heard so far for message seq originating
    // from node_id
    int priority(const int node_id, const int seq);

    void update_stats(const int node_id, const or_protocol_msgs::Header& header);

    void set_etx_map(const ETXMap& map) { link_etx = map; }
    FixedRoutingMap get_routing_map() { return routing_map; }

    // compute relays root should use for every possible flow in the network
    void update_routes(const int root);

    // query the routing table for the relays to use for the given src, dest
    RelayArray relays(const or_protocol_msgs::Header& header);

  private:
    // the state of each node in the network
    std::unordered_map<int, NodeState> node_states;

    // estimated ETX for each node in the network
    // NOTE we assume the network is connected in find_routes; link_etx_map must
    // be constructed accordingly
    ETXMap link_etx;

    // path for each node in the network
    FixedRoutingMap routing_map;

    // synchronize access to the routing and link ETX maps during reads/writes
    std::mutex routing_map_mutex, link_etx_mutex;
};


}  // namespace or_protocol


#endif
