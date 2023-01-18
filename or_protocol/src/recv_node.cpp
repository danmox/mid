#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/utils.h>
#include <or_protocol/or_node.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <std_msgs/UInt32.h>


volatile bool run = true;
rosbag::Bag bag;


void signal_handler(int s)
{
  ROS_INFO("[main] received shutdown signal %d", s);
  run = false;
}


void msg_cb(ros::Time recv_time, or_protocol_msgs::Packet& pkt, int node_id, int size)
{
  ROS_DEBUG("received message at %d with size %d", node_id, size);
  bag.write("recv_messages", recv_time, pkt);

  std_msgs::UInt32 seq;
  or_protocol::deserialize(seq, pkt.data.data(), pkt.data.size());
  bag.write("recv_seq_numbers", recv_time, seq);
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

  std::string log_file = "recv.bag";
  bag.open(log_file, rosbag::bagmode::Write);
  if (!bag.isOpen()) {
    ROS_FATAL("[main] failed to open log file: %s", log_file.c_str());
    exit(EXIT_FAILURE);
  } else {
    ROS_INFO("[main] logging received messages to %s", log_file.c_str());
  }

  or_protocol::ORNode or_node(argv[1], 4568);
  or_node.register_recv_func(msg_cb);
  while (run && or_node.is_running()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;
}
