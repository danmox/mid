#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include <fmt/format.h>
#include <or_protocol/or_protocol.h>
#include <or_protocol/utils.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Time.h>


volatile std::atomic<bool> run = true;
std::shared_ptr<or_protocol::ORProtocol> or_node;
int recv_msgs = 0;
std::vector<double> rtts_ms;


void ping_recv(ros::Time& recv_time, or_protocol_msgs::Packet& pkt, int size)
{
  if (pkt.header.msg_type != or_protocol_msgs::Header::PING_RES)
    return;

  std_msgs::Time send_stamp_msg;
  or_protocol::deserialize(send_stamp_msg, pkt.data.data(), pkt.data.size());

  std_msgs::Int32 msg_seq;
  uint32_t offset = ros::serialization::serializationLength(send_stamp_msg) + 4;
  or_protocol::deserialize(msg_seq, pkt.data.data() + offset, pkt.data.size() - offset);

  double ms = (recv_time - send_stamp_msg.data).toSec() * 1000;
  fmt::print("{} bytes from 192.168.0.{}: seq={} hops={} time={:.2f} ms\n",
             size, pkt.header.src_id, msg_seq.data, pkt.header.hops, ms);

  rtts_ms.push_back(ms);
  recv_msgs++;
}


void handler([[maybe_unused]]int s)
{
  run = false;
}


int main(int argc, char** argv)
{
  bool send_packets = true;
  int pkt_size = 64;
  if (argc == 2) {
    send_packets = false;
    fmt::print("[main] running in passive mode\n");
  } else if (argc == 3) {
  } else if (argc == 4) {
    pkt_size = std::stoi(argv[3]);
  } else {
    fmt::print("[main] usage: ping <ip address> [<node id> [<size>]]\n");
    return 0;
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  ros::Time::init();

  or_node.reset(new or_protocol::ORProtocol(argv[1], ping_recv));

  if (!send_packets) {
    while (run)
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return 0;
  }
  // build message
  or_protocol_msgs::Packet msg;
  msg.header.msg_type = or_protocol_msgs::Header::PING_REQ;
  msg.header.dest_id = std::stoi(argv[2]);
  msg.header.relays[0] = msg.header.dest_id;

  fmt::print("[main] sending {} byte pings to node {}\n", pkt_size, msg.header.dest_id);

  int sent_msgs = 0;
  ros::Time start = ros::Time::now();
  ros::Rate ping_rate(1);
  while (run && or_node->is_running()) {
    msg.data.clear();
    msg.header.hops = 0;

    std_msgs::Time now;
    now.data = ros::Time::now();
    or_protocol::pack_msg(msg, now);

    // the ping sequence and message sequence may not correspond if there are
    // more that one stream of data originating from the sending node
    std_msgs::Int32 ping_seq;
    ping_seq.data = sent_msgs;
    or_protocol::pack_msg(msg, ping_seq);

    // pad the message to the desired size
    const int pad = pkt_size - ros::serialization::serializationLength(msg) - 4;
    if (pad >= 0) {
      msg.data.insert(msg.data.end(), pad, 1);
    } else {
      fmt::print("[main] desired packet size of {} smaller than minimum packet size of {}\n", pkt_size, ros::serialization::serializationLength(msg) + 4);
      break;
    }

    bool set_relays = false;
    or_node->send(msg, set_relays);
    sent_msgs++;

    ping_rate.sleep();
  }

  if (sent_msgs == 0)
    return 0;

  fmt::print("\n--- 192.168.0.{} ping statistics ---\n", msg.header.dest_id);
  int total_time = (ros::Time::now() - start).toSec() * 1000;
  fmt::print("{} packets transmitted, {} packets received, {}% packet loss, {} ms\n",
             sent_msgs, recv_msgs, (sent_msgs - recv_msgs) * 100 / sent_msgs, total_time);

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

  fmt::print("rtt min/avg/max/std = {:.3f}/{:.3f}/{:.3f}/{:.3f} ms\n", min, mean, max, std);

  return 0;
}
