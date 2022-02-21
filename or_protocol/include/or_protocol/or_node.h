#ifndef OR_PROTOCOL_HOST_H_
#define OR_PROTOCOL_HOST_H_

#include <atomic>
#include <netinet/in.h>
#include <string>
#include <thread>

namespace or_protocol {


class ORNode
{
  public:
    std::atomic<bool> run;

    ORNode(std::string _IP, int _port);
    ~ORNode();

  private:
    int recv_sockfd, port;
    std::string IP;

    // IPv4 address information about this node
    sockaddr_in my_sa;

    std::thread send_thread, recv_thread;

    void send_loop();
    void recv_loop();
};


} // namespace or_protocol


#endif
