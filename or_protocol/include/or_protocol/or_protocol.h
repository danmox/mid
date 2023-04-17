#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>

#include <or_protocol/bcast_socket.h>
#include <or_protocol/constants.h>
#include <or_protocol/network_state.h>
#include <or_protocol/utils.h>
#include <rosbag/bag.h>


namespace or_protocol {


class ORProtocol
{
  public:
    // initializes ORNode with a unique IPv4 address
    ORProtocol(std::string _IP);

    // destructor required for thread cleanup logic
    ~ORProtocol();

    // the send interface between higher level application nodes and ORNode
    bool send(or_protocol_msgs::Packet& msg, const bool set_relays = true);

    // checks if this node and the underlying socket is ready for use
    bool is_running() const { return run && bcast_socket->is_running(); }

    // TODO implement: shut down the node
    void shutdown();

    template <typename T>
    void log_ros_msg(const std::string &topic, const ros::Time &time, const T &msg)
    {
      static const std::string prefix = "/node" + std::to_string(node_id) + "/";
      if (bag.isOpen()) {
        std::lock_guard<std::mutex> lock(log_mutex);
        bag.write(prefix + topic, time, msg);
      } else {
        OR_ERROR("cannot write msg to bag! bag not open!");
      }
    }

    // the receive interface between ORNode and higher level application nodes
    void register_recv_func(msg_recv_func fcn);

    friend class ORProtocolTest;

  private:
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

    // a class keeping track of the current state of the network, including
    // received messages and estimated ETX values for each node
    NetworkState network_state;

    // the main packet queue processed by this node; incoming messages are
    // pushed onto this queue for processing by the process thread
    SafeFIFOQueue<std::shared_ptr<PacketQueueItem>> packet_queue;

    // sequence numbers of unACKed reliable messages
    std::unordered_set<uint32_t> retransmission_set;

    // processing the packet queue, logging, and updating the retransmission set
    // occur across threads and must be protected
    std::mutex log_mutex, retrans_mutex;

    // packet processing thread
    std::thread process_thread;

    // threads for transmitting beacons frames at a regular interval and
    // processing received beacon frames
    std::thread beacon_tx_thread, beacon_rx_thread;

    // thread for recomputing the routing table
    std::thread routing_thead;

    // bagfile used for logging packet statistics
    rosbag::Bag bag;

    // a wrapper for BCastSocket::send(...)
    bool send(const char* buff, size_t size);

    // a function called by the underlying BCastSocket instance for each
    // incoming message
    void recv(buffer_ptr& buff_ptr, size_t size);

    // main thread processing packet_queue and making forwarding decisions
    void process_packets();

    // all received beacon frames are pushed onto a queue for processing by a
    // separate thread to avoid blocking in the main packet processing thread
    void process_beacons();

    // thread for transmitting beacon frames at a regular interval
    void transmit_beacons();

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
      return network_state.priority(header.src_id, header.seq);
    };
};


class ORProtocolTest
{
  public:
    void initializeORNode(ORProtocol& node, const ETXMap& etx_map) {
      node.network_state.set_etx_map(etx_map);
      node.network_state.update_routes(node.node_id);
    }
};


}  // namespace or_protocol


#endif
