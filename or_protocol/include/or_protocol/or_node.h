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
    ORNode(std::string _IP, int _port);
    bool send(or_protocol_msgs::Packet& msg, bool fill_src = true);
    bool run() const { return bcast_socket->run; }
    void register_recv_func(msg_recv_func fcn);

  private:
    std::shared_ptr<BCastSocket> bcast_socket;
    int node_id;
    int seq = 0;
    msg_recv_func recv_handle = nullptr;
    std::unordered_map<int, NodeState> node_states;

    bool send(const char* buff, size_t size);
    void recv(buffer_ptr& buff_ptr, size_t size);

    void print_msg_info(std::string msg,
                        const or_protocol_msgs::Header& header,
                        int size,
                        bool total);
};


}  // namespace or_protocol


#endif
