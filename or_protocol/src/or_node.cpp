#include <chrono>
#include <iostream>

#include <or_protocol/or_node.h>


namespace or_protocol {


ORNode::ORNode(std::string _IP, int _port)
{
  // initialize BCastSocket and register recv function handle
  using namespace std::placeholders;
  recv_function fcn = std::bind(&ORNode::recv, this, _1, _2);
  bcast_socket.reset(new BCastSocket(_IP, _port, fcn));

  run = true;
}


void ORNode::recv(char* buff, int size)
{
  std::cout << "[ORNode] received " << size << " bytes: " << buff << std::endl;
}


void ORNode::main_loop()
{
  std::string msg = "hello from " + bcast_socket->getIP();
  const char* msg_str = msg.c_str();

  // TODO devise better way of shutting down
  while (run && bcast_socket->run) {
    bcast_socket->send(msg_str, msg.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
}


} // namespace or_protocol
