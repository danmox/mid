#include <chrono>
#include <csignal>
#include <iostream>

#include <or_protocol/or_protocol.h>


volatile bool run = true;


void signal_handler(int s)
{
  ROS_DEBUG("[main] received shutdown signal %d", s);
  run = false;
}


int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cout << "[main] usage: or_test <ip address>" << std::endl;
    return 0;
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = signal_handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  or_protocol::ORProtocol or_node(argv[1]);

  while (run && or_node.is_running())
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ROS_INFO("[main] sleeping for 4 seconds to allow all messages to transmit");
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  ROS_INFO("[main] exiting");

  return 0;
}
