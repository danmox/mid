#include <chrono>
#include <iostream>

#include <or_protocol/or_node.h>


namespace or_protocol {


ORNode::ORNode(std::string _IP, int _port)
{
  // TODO use interface name instead? enabling automatic IP address fetching

  // initialize BCastSocket and register recv function handle
  using namespace std::placeholders;
  recv_function fcn = std::bind(&ORNode::recv, this, _1, _2);
  bcast_socket.reset(new BCastSocket(_IP, _port, fcn));

  run = true;
}


void ORNode::send(const char* buff, int size)
{
  bcast_socket->send(buff, size);
}


void ORNode::recv(char* buff, int size)
{
  std::cout << "[ORNode] received " << size << " bytes: " << buff << std::endl;
}


void ORNode::send_loop() {
  std::string msg = "hello from " + bcast_socket->getIP();
  const char *msg_str = msg.c_str();

  // TODO devise better way of shutting down
  while (run && bcast_socket->run) {
    send(msg_str, msg.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
}


void ORNode::recv_loop()
{
  while (run && bcast_socket->run) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}


} // namespace or_protocol
