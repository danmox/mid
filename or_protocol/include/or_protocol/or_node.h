#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <atomic>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <thread>

#include <or_protocol/bcast_socket.h>
#include <or_protocol_msgs/Packet.h>


namespace or_protocol {


class ORNode
{
  public:
    volatile std::atomic<bool> run;

    ORNode(std::string _IP, int _port);
    void send_loop(or_protocol_msgs::PacketPtr& msg);
    void recv_loop();
    void sig_handler(int s);

  private:
    std::shared_ptr<BCastSocket> bcast_socket;
    int node_id;

    void recv(char* buff, size_t size);
    bool send(const or_protocol_msgs::PacketConstPtr& msg);
    bool send(const char* buff, size_t size);
};


} // namespace or_protocol


#endif
