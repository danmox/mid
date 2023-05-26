#ifndef OR_PROTOCOL_OR_PROTOCOL_H_
#define OR_PROTOCOL_OR_PROTOCOL_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <stdio.h>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_set>

#include <or_protocol/bcast_socket.h>
#include <or_protocol/constants.h>
#include <or_protocol/network_state.h>
#include <or_protocol/utils.h>


namespace or_protocol {


using or_protocol_msgs::Header;


class ORProtocol
{
  public:
    // initializes ORNode with a unique IPv4 address
    ORProtocol(std::string _IP, msg_recv_func msg_cb = nullptr);

    // destructor required for thread cleanup logic
    ~ORProtocol();

    // the send interface between higher level application nodes and ORNode
    bool send(or_protocol_msgs::Packet& msg, const bool set_relays = true);

    // checks if this node and the underlying socket is ready for use
    bool is_running() const { return run && bcast_socket->is_running(); }

    // TODO implement: shut down the node
    void shutdown();

    int get_node_id() const { return node_id; }

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
    msg_recv_func recv_handle;

    // a class keeping track of the current state of the network, including
    // received messages and estimated ETX values for each node
    NetworkState network_state;

    // the main packet queue processed by this node; incoming messages are
    // pushed onto this queue for processing by the process thread
    // TODO check for overflow
    SafeFIFOQueue<PacketQueueItemPtr> packet_queue;

    // sequence numbers of unACKed reliable messages
    std::unordered_set<uint32_t> retransmission_set;

    // logging, updating the retransmission set, and passing messages to the
    // application occur across threads and must be protected
    std::mutex log_mutex, retrans_mutex, app_mutex;

    // packet processing thread
    std::thread process_thread;

    // threads for transmitting beacons frames at a regular interval and
    // processing received beacon frames
    std::thread beacon_tx_thread, beacon_rx_thread;

    // thread for recomputing the routing table
    std::thread routing_thread;

    // safe queue for log information to be processed by logging thread
    typedef std::tuple<Header, PacketAction, int, ros::Time, std::string> LogQueueItem;
    SafeFIFOQueue<LogQueueItem> log_queue;

    // thread for logging protocol decisions in plain text
    std::thread log_thread;

    // file used for packet routing and transmission logging
    std::FILE* log_file;

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

    // thread for recomputing the routing table
    void compute_routes();

    // thread for processing the log_queue
    void process_log_queue();

    // helper functions for logging
    void log_event(const Header& header,
                   const PacketAction action,
                   const int size,
                   const ros::Time& time,
                   const std::string& msg = "");

    void log_event(const PacketQueueItemPtr& item,
                   const PacketAction action,
                   const ros::Time& time,
                   const std::string& msg = "");

    // helper function returning the unique message sequence number to be
    // embedded in outgoing message headers
    uint32_t getSeqNum() { return seq++; }

    // helper function returning the priority of the most recent received
    // message associated with header
    int msg_priority(const Header& header)
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
