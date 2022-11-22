#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <atomic>
#include <memory>
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


class ORNode
{
  public:
    // initializes ORNode with an IPv4 address (must be unique to the current
    // node) and a port to transmit to (must be shared by all nodes in the
    // network)
    ORNode(std::string _IP, int _port);

    // the send interface between higher level application nodes and ORNode
    bool send(or_protocol_msgs::Packet& msg, bool fill_src = true);

    // checks if the underlying socket is ready for use
    bool run() const { return bcast_socket->run; }

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

    // a function called by the process thread for messages for which the
    // current node is the intended destination
    msg_recv_func recv_handle = nullptr;

    // a map of node states that keep track of messages received, allowing for
    // quickly determining if a message has already been received and processed,
    // as well as network statistics for each node
    std::unordered_map<int, NodeState> node_states;

    // a wrapper for BCastSocket::send(...)
    bool send(const char* buff, size_t size);

    // a function called by the underlying BCastSocket instance for each
    // incoming message
    void recv(buffer_ptr& buff_ptr, size_t size);

    // a helper function for logging information about a message
    void print_msg_info(std::string msg,
                        const or_protocol_msgs::Header& header,
                        int size,
                        bool total);
};


}  // namespace or_protocol


#endif
