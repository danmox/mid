#ifndef OR_PROTOCOL_NODE_STATE_H_
#define OR_PROTOCOL_NODE_STATE_H_


#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include <or_protocol/constants.h>
#include <or_protocol/types.h>
#include <or_protocol/utils.h>

#include <geometry_msgs/PoseStamped.h>
#include <or_protocol_msgs/Header.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <or_protocol_msgs/RoutingTable.h>


namespace or_protocol {


using or_protocol_msgs::ETXEntry;


struct AttemptPriorityPair
{
    unsigned int attempt;
    unsigned int priority;
};


struct SeqAttemptStamped
{
    uint32_t seq;
    unsigned int attempt;
    double stamp;
};


struct MsgStatus
{
    bool is_new_msg;
    bool is_new_seq;
};


struct ETXEntryStamped
{
    ETXEntry entry;
    ros::Time stamp;
};


class NodeState
{
  public:
    // update the received message queue used for determining if a message has:
    // 1) already been received and doesn't need to be processed again or 2) has
    // already been relayed by a higher priority node; returns if the message is
    // new
    MsgStatus update_queue(const or_protocol_msgs::Header& header, const double stamp);

    // updated the received message queue when an ACK has been received (i.e.
    // zero out the priority simulating the reception of a relay a higher
    // priority node)
    void ack_msg(const uint32_t seq, const double stamp);

    // return the priority of a message (assumes a message with the same source
    // and sequence number has already been added to the queue)
    int priority(const int seq);

    // update delivery probability estimate
    void update_link_state(const PacketQueueItemPtr& beacon = nullptr);

    ETXEntry get_etx_entry(const int dest) const;
    ros::Time get_etx_stamp() const { return last_beacon_stamp; }

  private:
    // the history of messages received kept as a deque for quickly determining
    // the oldest message in the queue and as a map for quickly determining if a
    // message has been received before and if so the highest received priority;
    // these two data structures are kept synchronized
    std::unordered_map<uint32_t, AttemptPriorityPair> msg_hist_map;
    std::deque<SeqAttemptStamped> msg_hist_deque;

    // variables for estimating the delivery probability from this node
    ros::Time last_beacon_stamp{0};
    MovingAverage<uint8_t, ETX_BUFFER_LEN> delivery_probability;
    uint16_t etx_seq{0};
};


// TODO clean up these definitions / move to a pair/tuple key
typedef std::unordered_map<int, std::unordered_map<int, double>> ETXMap;
typedef std::unordered_map<int, std::unordered_map<int, ETXEntryStamped>> ETXEntryMap;
typedef std::unordered_map<int, std::vector<int>> IntVectorMap;
typedef or_protocol_msgs::Header::_relays_type RelayArray;
typedef std::unordered_map<int, RelayArray> IntArrayMap;
typedef std::unordered_map<int, std::unordered_map<int, IntVectorMap>> VariableRoutingMap;
typedef std::unordered_map<int, std::unordered_map<int, IntArrayMap>> FixedRoutingMap;

const int MAX_RELAY_COUNT = RelayArray::max_size();


class NetworkState
{
  public:
    NetworkState() : new_pose(false) {}

    // update the received message queue for the given node
    MsgStatus update_queue(const PacketQueueItemPtr& item);

    // update the received message queue when an ACK has been received
    void ack_msg(const int dest_id, const uint32_t seq, const double stamp);

    // returns the highest priority heard so far for message seq originating
    // from node_id
    int priority(const int node_id, const int seq);

    // add beacon (NetworkStatus) frame to queue
    void push_beacon_queue(const PacketQueueItemPtr& item) { beacon_queue.push(item); }

    // process beacon (NetworkStatus) frames in queue
    void process_beacon_queue(const int root);

    // generate beacon (NetworkStatus) message to be sent to other nodes
    or_protocol_msgs::NetworkStatus::Ptr generate_beacon();

    // NOTE ONLY FOR TESTING NOT THREAD SAFE
    void set_etx_map(const ETXMap& map);
    FixedRoutingMap get_routing_map() { return routing_map; }

    // returns the routing map in the form of a ROS message
    or_protocol_msgs::RoutingTable::Ptr get_routing_table_msg();

    // compute relays every possible flow in the network involving root_id
    void update_routes(const int root);

    // query the routing table for the relays to use for the given src, dest
    RelayArray relays(const or_protocol_msgs::Header& header, const int node_id);

    // update this node's position estimate to be shared in the status beacon
    void update_pose(const geometry_msgs::PoseStampedConstPtr& msg, const int id);

  private:
    // indicating the received pose is the first since the last beacon was sent
    volatile bool new_pose;

    // the message history of each node in the network used for determining
    // if/when to relay received messages
    std::unordered_map<int, NodeState> node_states;

    // beacon (NetworkState) frame queue; frames are pushed here by the main
    // packet thread for eventual processing in a separate worker thread
    SafeFIFOQueue<PacketQueueItemPtr> beacon_queue;

    // estimated ETX for each node in the network computed via beaconing
    ETXEntryMap link_etx_table;

    // positions of each robot in the network in a common reference frame
    std::unordered_map<int, or_protocol_msgs::Point> node_positions;

    // path for each node in the network
    FixedRoutingMap routing_map;

    // synchronize access to the routing and link ETX maps during reads/writes
    std::mutex routing_map_mutex, status_mutex;

    // list of node ids for convenience
    std::unordered_set<int> node_ids;
};


}  // namespace or_protocol


#endif
