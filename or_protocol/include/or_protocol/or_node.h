#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <atomic>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <thread>

#include <or_protocol/bcast_socket.h>


namespace or_protocol {


class ORNode
{
  public:
    volatile std::atomic<bool> run;

    ORNode(std::string _IP, int _port);
    void main_loop();
    void sig_handler(int s);

  private:
    std::shared_ptr<BCastSocket> bcast_socket;

    void recv(char* buff, int size);
};


} // namespace or_protocol


#endif
