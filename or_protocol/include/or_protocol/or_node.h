#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <atomic>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <thread>

#include <or_protocol/bcast_socket.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/console.h>


namespace or_protocol {


#define OR_INFO(fmt, ...) ROS_INFO("[ORNode] " fmt, ##__VA_ARGS__)
#define OR_DEBUG(fmt, ...) ROS_DEBUG("[ORNode] " fmt, ##__VA_ARGS__)


class ORNode
{
  public:
    ORNode(std::string _IP, int _port);
    bool send(or_protocol_msgs::Packet &msg, bool fill_src = true);
    bool send(const or_protocol_msgs::PacketConstPtr &msg);
    bool send(const char *buff, size_t size);
    bool run() const {return bcast_socket->run;}

  private:
    std::shared_ptr<BCastSocket> bcast_socket;
    int node_id;
    int seq = 0;

    void recv(char* buff, size_t size);
    void print_msg_info(std::string msg, const or_protocol_msgs::Packet& packet);
};


} // namespace or_protocol


#endif
