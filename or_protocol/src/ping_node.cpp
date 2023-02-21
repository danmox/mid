#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/or_node.h>
#include <or_protocol/utils.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Time.h>


volatile bool run = true;
int recv_msgs = 0;
std::vector<double> rtts_ms;

void ping_recv(ros::Time recv_time, or_protocol_msgs::Packet& msg, int node_id, int bytes)
{
  if (msg.header.dest_id != node_id ||
      msg.header.msg_type != or_protocol_msgs::Header::PING_RES)
    return;

  std_msgs::Time send_stamp_msg;
  or_protocol::deserialize(send_stamp_msg, msg.data.data(), msg.data.size());

  std_msgs::Int32 msg_seq;
  uint32_t offset = ros::serialization::serializationLength(send_stamp_msg) + 4;
  or_protocol::deserialize(msg_seq, msg.data.data() + offset, msg.data.size() - offset);

  double ms = (recv_time - send_stamp_msg.data).toSec() * 1000;
  printf("%d bytes from 192.168.0.%d: seq=%d, hops=%d, time=%.2f ms\n",
         bytes, msg.header.src_id, msg_seq.data, msg.header.hops, ms);

  rtts_ms.push_back(ms);
  recv_msgs++;
}


void handler(int s)
{
  ROS_DEBUG("[main] received shutdown signal %d", s);
  run = false;
}


int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cout << "[main] usage: ping <ip address> <node id>" << std::endl;
    return 0;
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  ros::Time::init();

  or_protocol::ORNode or_node(argv[1]);
  or_node.register_recv_func(ping_recv);

  // build message
  or_protocol_msgs::Packet msg;
  msg.header.msg_type = or_protocol_msgs::Header::PING_REQ;
  msg.header.dest_id = std::stoi(argv[2]);
  msg.header.relays[0] = 2;
  msg.header.relays[1] = 3;
  msg.header.relays[2] = 4;

  int sent_msgs = 0;
  ros::Time start = ros::Time::now();
  while (run && or_node.is_running()) {
    msg.data.clear();
    msg.header.hops = 0;

    std_msgs::Time now;
    now.data = ros::Time::now();
    or_protocol::pack_msg(msg, now);

    // the ping sequence and message sequence may not correspond if there are
    // more that 1 datastream originating from the sending node
    std_msgs::Int32 ping_seq;
    ping_seq.data = sent_msgs;
    or_protocol::pack_msg(msg, ping_seq);

    // pad the message so that it's the same size as a normal ping (64 bytes)
    msg.data.insert(msg.data.end(), 22, 1);

    or_node.send(msg);
    sent_msgs++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  printf("\n--- 192.168.0.%d ping statistics ---\n", msg.header.dest_id);
  int total_time = (ros::Time::now() - start).toSec() * 1000;
  printf("%d packets transmitted, %d packets received, %d%% packet loss, %d ms\n",
         sent_msgs, recv_msgs, (sent_msgs - recv_msgs) * 100 / sent_msgs,
         total_time);

  if (rtts_ms.size() == 0)
    return 0;

  double min = *std::min_element(rtts_ms.begin(), rtts_ms.end());
  double max = *std::max_element(rtts_ms.begin(), rtts_ms.end());

  double sum = 0;
  for (double num : rtts_ms)
    sum += num;
  double mean = sum / rtts_ms.size();

  double sq_sum = 0;
  for (double num : rtts_ms)
    sq_sum += (num - mean) * (num - mean);
  double std = sqrt(sq_sum / rtts_ms.size());

  printf("rtt min/avg/max/std = %.3f/%.3f/%.3f/%.3f ms\n", min, mean, max, std);

  return 0;
}
