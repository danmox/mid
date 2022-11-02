#ifndef OR_PROTOCOL_BCAST_SOCKET_H_
#define OR_PROTOCOL_BCAST_SOCKET_H_

#include <atomic>
#include <functional>
#include <netinet/in.h>
#include <string>
#include <thread>


namespace or_protocol {


typedef std::function<void(char*,int)> recv_function;


class BCastSocket
{
  public:
    volatile std::atomic<bool> run;

    BCastSocket(std::string _IP, int _port, recv_function _recv_handle);
    ~BCastSocket();

    std::string getIP() {return my_IP;}
    bool send(const char *buff, int size);

  private:
    int recv_sockfd, send_sockfd, port;
    std::string my_IP, bcast_IP;

    // function handle to call when receiving messages
    recv_function recv_handle;

    // IPv4 address information about this node
    sockaddr_in my_sa, bcast_addr;

    std::thread recv_thread;

    bool init_send_socket();
    void recv_loop();
};


} // namespace or_protocol


#endif
