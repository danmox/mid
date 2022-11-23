#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <thread>
#include <unordered_map>

#include <or_protocol/bcast_socket.h>
#include <or_protocol/node_state.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/console.h>


namespace or_protocol {


#define OR_DEBUG(fmt, ...) ROS_DEBUG("[ORNode] " fmt, ##__VA_ARGS__)
#define OR_INFO(fmt, ...) ROS_INFO("[ORNode] " fmt, ##__VA_ARGS__)
#define OR_ERROR(fmt, ...) ROS_ERROR("[ORNode] " fmt, ##__VA_ARGS__)


typedef std::function<void(or_protocol_msgs::Packet&, int, int)> msg_recv_func;


void update_msg_header(char*, const or_protocol_msgs::Header&);


template <typename M>
void deserialize(M& msg, uint8_t* buff, int size, bool size_prefix = true)
{
  // NOTE the total size of the serialized message is often prepended as an
  // int32_t: http://wiki.ros.org/msg#Fields
  int offset = size_prefix ? 4 : 0;
  ros::serialization::IStream s(buff + offset, size - offset);
  ros::serialization::deserialize(s, msg);
}


struct PacketQueueItem
{
    buffer_ptr buff_ptr;
    size_t size;
    ros::Time min_transmit_time;
    bool processed;

    PacketQueueItem(buffer_ptr& _buff_ptr, size_t _size) :
      buff_ptr(_buff_ptr), size(_size), min_transmit_time(0), processed(false)
    {}

    char* buffer() { return buff_ptr.get(); }
};


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
    bool is_running() const { return run && bcast_socket->run; }

    // TODO implement: shut down the node
    void shutdown();

    // the receive interface between ORNode and higher level application nodes
    void register_recv_func(msg_recv_func fcn);

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
    int seq = 0;

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

    std::mutex queue_mutex;

    // packet processing thread
    std::thread process_thread;

    // a wrapper for BCastSocket::send(...)
    bool send(const char* buff, size_t size);

    // a function called by the underlying BCastSocket instance for each
    // incoming message
    void recv(buffer_ptr& buff_ptr, size_t size);

    // main thread processing packet_queue and making forwarding decisions
    void process_packets();

    // a helper function for logging information about a message
    void print_msg_info(std::string msg,
                        const or_protocol_msgs::Header& header,
                        int size,
                        bool total);
};


}  // namespace or_protocol


#endif
