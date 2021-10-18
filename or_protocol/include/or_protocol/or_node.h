#ifndef OR_PROTOCOL_HOST_H_
#define OR_PROTOCOL_HOST_H_


#include <atomic>
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
    std::thread send_thread, recv_thread;

    void send_loop();
    void recv_loop();
};


} // namespace or_protocol


#endif
