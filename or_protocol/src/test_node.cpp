#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/or_node.h>


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
  or_node->main_loop();

  return 0;
}
