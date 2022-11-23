#ifndef OR_PROTOCOL_BCAST_SOCKET_H_
#define OR_PROTOCOL_BCAST_SOCKET_H_

#include <atomic>
#include <functional>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <thread>


namespace or_protocol {


// TODO switch to std::unique_ptr for better performance?
typedef std::function<void(std::shared_ptr<char[]>&, int)> buff_recv_func;
typedef std::shared_ptr<char[]> buffer_ptr;


class BCastSocket
{
  public:
    // initializes BCastSocket with an IPv4 address, port to transmit on, and a
    // function handle to call when new messages are received -- the provided
    // IPv4 address is used to initialize sockets that send/receive UDP messages
    // on the associated network broadcast channel.
    BCastSocket(std::string _IP, int _port, buff_recv_func _recv_handle);

    // destructor required for socket and thread cleanup
    ~BCastSocket();

    // sends 'size' bytes pointed to by 'buff' to network broadcast channel
    bool send(const char* buff, int size);

    // checks if the send / receive sockets are ready for use
    bool is_running() const { return run; }

  private:
    // internal state used to gracefully signal a shutdown to running threads
    volatile std::atomic<bool> run;

    // the receive and send socket file descriptors and their associated port
    int recv_sockfd, send_sockfd, port;

    // the IP address of this node and the associated broadcast IP address
    std::string my_IP, bcast_IP;

    // the sockaddr_in versions of the IPv4 address strings above
    sockaddr_in my_sa, bcast_addr;

    // function handle to call when messages are received
    buff_recv_func recv_handle;

    // the receive message thread, which blocks waiting for new messages
    std::thread recv_thread;

    // function called in recv_thread that handles initialization of the receive
    // socket as well as waiting for new messages and passing them off to the
    // higher level applications
    void recv_loop();

    // function initializing the send socket
    bool init_send_socket();
};


}  // namespace or_protocol


#endif
