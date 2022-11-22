#ifndef OR_PROTOCOL_BCAST_SOCKET_H_
#define OR_PROTOCOL_BCAST_SOCKET_H_

#include <atomic>
#include <functional>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <thread>


namespace or_protocol {


typedef std::function<void(std::shared_ptr<char[]>&, int)> buff_recv_func;
typedef std::shared_ptr<char[]> buffer_ptr;


class BCastSocket
{
  public:
    volatile std::atomic<bool> run;

    BCastSocket(std::string _IP, int _port, buff_recv_func _recv_handle);
    ~BCastSocket();

    std::string getIP() { return my_IP; }
    bool send(const char* buff, int size);

  private:
    int recv_sockfd, send_sockfd, port;
    std::string my_IP, bcast_IP;

    // function handle to call when receiving messages
    buff_recv_func recv_handle;

    // IPv4 address information about this node
    sockaddr_in my_sa, bcast_addr;

    std::thread recv_thread;

    bool init_send_socket();
    void recv_loop();
};


}  // namespace or_protocol


#endif
