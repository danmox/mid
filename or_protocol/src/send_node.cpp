#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/or_node.h>
#include <or_protocol/utils.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <std_msgs/UInt32.h>


volatile bool run = true;


void signal_handler(int s)
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
  siginthandler.sa_handler = signal_handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  std::string log_file = "send.bag";
  rosbag::Bag bag;
  bag.open(log_file, rosbag::bagmode::Write);
  if (!bag.isOpen()) {
    ROS_FATAL("[main] failed to open log file: %s", log_file.c_str());
    exit(EXIT_FAILURE);
  } else {
    ROS_INFO("[main] logging received messages to %s", log_file.c_str());
  }

  or_protocol::ORNode or_node(argv[1]);

  // build message
  or_protocol_msgs::Packet msg;
  msg.header.msg_type = or_protocol_msgs::Header::PAYLOAD;
  msg.header.dest_id = 46;
  msg.header.relays = {45, 44, 42, 41};
  msg.header.reliable = true;

  std_msgs::UInt32 ping_seq;
  ping_seq.data = 0;
  size_t total_msgs = 200;
  ROS_INFO("[main] sending %ld messages", total_msgs);
  while (run && or_node.is_running() && ping_seq.data < total_msgs) {
    msg.header.hops = 0;
    msg.data.clear();
    or_protocol::pack_msg(msg, ping_seq);
    msg.data.insert(msg.data.end(), 92, 1);  // pad the array to 100 elements
    or_node.send(msg);
    bag.write("sent_seq_numbers", ros::Time::now(), ping_seq);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ping_seq.data++;
  }

  ROS_INFO("[main] sleeping for 4 seconds to allow all messages to transmit");
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  ROS_INFO("[main] send test complete");

  return 0;
}
