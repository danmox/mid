#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/or_node.h>
#include <or_protocol_msgs/Packet.h>


std::shared_ptr<or_protocol::ORNode> or_node;


void handler(int s)
{
  std::cout << "[main] received shutdown signal (" << s << ")" << std::endl;
  or_node->run = false;
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

  or_node.reset(new or_protocol::ORNode(argv[1], 4568));

  // build message
  or_protocol_msgs::PacketPtr msg(new or_protocol_msgs::Packet);
  msg->dest_id = 2;
  msg->seq = 1;
  msg->data = std::vector<uint8_t>{'h','e','l','l','o'};

  or_node->send_loop(msg);

  return 0;
}
