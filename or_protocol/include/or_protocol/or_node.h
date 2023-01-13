#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>

#include <or_protocol/bcast_socket.h>
#include <or_protocol/node_state.h>
#include <rosbag/bag.h>


namespace or_protocol {


class ORNode
{
  public:
    // initializes ORNode with an IPv4 address (must be unique to the current
    // node) and a port to transmit to (must be shared by all nodes in the
    // network)
    ORNode(std::string _IP, int _port);

    // destructor required for thread cleanup logic
    ~ORNode();

    // the send interface between higher level application nodes and ORNode
    bool send(or_protocol_msgs::Packet& msg, bool fill_src = true);

    // checks if this node and the underlying socket is ready for use
    bool is_running() const { return run && bcast_socket->is_running(); }

    // TODO implement: shut down the node
    void shutdown();

    // the receive interface between ORNode and higher level application nodes
    void register_recv_func(msg_recv_func fcn);

  private:
    // minimum unit of delay for enforcing relay priority in nanoseconds
    static const uint32_t UNIT_DELAY = 10000000;  // 10ms

    // duration of time in nanoseconds to wait for an ACK before re-transmitting
    // a reliable packet, taking into account the worst case forwarding path for
    // the packet and ACK
    static const uint32_t RETRY_DELAY = 13 * UNIT_DELAY;

    // maximum number of times a reliable packet should be re-transmitted
    static const int MAX_RETRY_COUNT = 2;

    // socket used for broadcasting to peers, implementing the lower level
    // broadcast socket setup / cleanup and usage
    std::shared_ptr<BCastSocket> bcast_socket;

    // node id associated with the current instance, typically identifying the
    // last digits of the corresponding IPv4 address identified by the subnet
    // mask 0.0.0.255
    int node_id;

    // a sequence number uniquely identifying messages that originating from
    // this node
    volatile std::atomic<uint32_t> seq = 0;

    // internal state used to gracefully signal a shutdown to running threads
    volatile std::atomic<bool> run;

    // a function called by the process thread for messages for which the
    // current node is the intended destination
    msg_recv_func recv_handle = nullptr;

    // a map of node states that keep track of messages received, allowing for
    // quickly determining if a message has already been received and processed,
    // as well as network statistics for each node
    std::unordered_map<int, NodeState> node_states;

    // the main packet queue processed by this node; incoming messages are
    // pushed onto this queue for processing by the process thread
    std::deque<std::shared_ptr<PacketQueueItem>> packet_queue;

    // sequence numbers of unACKed reliable messages
    // TODO make thread safe
    std::unordered_set<uint32_t> retransmission_set;

    // processing the packet queue, logging, and updating the retransmission set
    // occur across threads and must be protected
    std::mutex queue_mutex, log_mutex, retrans_mutex;

    // packet processing thread
    std::thread process_thread;

    // bagfile used for logging packet statistics
    rosbag::Bag bag;

    // a wrapper for BCastSocket::send(...)
    bool send(const char* buff, size_t size);

    // a function called by the underlying BCastSocket instance for each
    // incoming message
    void recv(buffer_ptr& buff_ptr, size_t size);

    // main thread processing packet_queue and making forwarding decisions
    void process_packets();

    // a helper function for logging information about a message
    void print_msg_info(const std::string& msg,
                        const or_protocol_msgs::Header& header,
                        int size,
                        bool total = true);

    // helper function for logging messages
    void log_message(const or_protocol_msgs::Header& header,
                     const int action,
                     const int size,
                     const ros::Time& time);

    // helper function returning the unique message sequence number to be
    // embedded in outgoing message headers
    inline uint32_t getSeqNum() { return seq++; }

    // helper function returning the priority of the most recent received
    // message associated with header
    inline int msg_priority(const or_protocol_msgs::Header& header)
    {
      return node_states[header.src_id].priority(header.seq);
    };

    inline void push_packet_queue(PacketQueueItemPtr& ptr)
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      packet_queue.push_back(ptr);
    }
};


}  // namespace or_protocol


#endif
