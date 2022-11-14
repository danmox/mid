#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/or_node.h>
#include <or_protocol_msgs/Packet.h>


volatile bool run = true;


void handler(int s)
{
  std::cout << "[main] received shutdown signal (" << s << ")" << std::endl;
  run = false;
}


int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cout << "[main] usage: or_test <ip address>" << std::endl;
    return 0;
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  or_protocol::ORNode or_node(argv[1], 4568);

  // build message
  or_protocol_msgs::Packet msg;
  msg.msg_type = or_protocol_msgs::Packet::PAYLOAD;
  msg.dest_id = 100;
  msg.data = std::vector<uint8_t>(200, 1);

  std::cout << "starting send loop" << std::endl;
  while (run && or_node.run()) {
    or_node.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
