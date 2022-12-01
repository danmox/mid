#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/or_node.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/console.h>


volatile bool run = true;


void handler(int s)
{
  ROS_INFO("[main] received shutdown signal %d", s);
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
  msg.header.msg_type = or_protocol_msgs::Header::PAYLOAD;
  msg.header.dest_id = 100;
  msg.data = std::vector<uint8_t>(200, 1);

  while (run && or_node.is_running()) {
    or_node.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
