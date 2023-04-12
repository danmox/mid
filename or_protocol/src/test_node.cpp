#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <yaml-cpp/yaml.h>

#include <or_protocol/or_protocol.h>
#include <or_protocol/utils.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <std_msgs/UInt32.h>


volatile bool run = true;
rosbag::Bag bag;
std::mutex bag_mutex;


namespace fs = std::filesystem;
using ros::serialization::serializationLength;


struct FlowSpec
{
    or_protocol_msgs::Packet packet;
    std_msgs::UInt32 seq;
    int total_size;
};


void signal_handler(int s)
{
  ROS_INFO("[main] received shutdown signal %d", s);
  run = false;
}


void msg_cb(ros::Time recv_time, or_protocol_msgs::Packet& pkt, int node_id, int size)
{
  ROS_DEBUG("received message at %d with size %d", node_id, size);

  std_msgs::UInt32 seq;
  or_protocol::deserialize(seq, pkt.data.data(), pkt.data.size());

  std::lock_guard<std::mutex> lock(bag_mutex);
  bag.write("recv_seq_numbers", recv_time, seq);
}


int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cout << "[main] usage: or_test <ip address> <sample.yaml>" << std::endl;
    return 0;
  }

  // load flows and etx table from yaml
  fs::path sample_path(argv[2]);
  if (!fs::is_regular_file(sample_path)) {
    std::cout << "[main] " << sample_path << " is not a file" << std::endl;
    return 0;
  }
  YAML::Node sample = YAML::LoadFile(sample_path.string());
  or_protocol::ETXMap sample_link_etx;
  if (!sample["link_etx"] || !sample["flows"]) {
    std::cout << "[main] 'link_etx' and 'flows' must be specified" << std::endl;
    return 0;
  }
  for (const auto& node : sample["link_etx"]) {
    const int& tx_node = node.first.as<int>();
    const YAML::Node& map_node = node.second;

    std::unordered_map<int, double> inner_map;
    for (auto& inner_node : map_node) {
      const int& rx_node = inner_node.first.as<int>();
      const double& etx = inner_node.second.as<double>();
      inner_map.emplace(rx_node, etx);
    }

    sample_link_etx.emplace(tx_node, inner_map);
  }
  std::unordered_map<int, std::unordered_map<int, FlowSpec>> flows;
  for (const auto& node : sample["flows"]) {
    int src = node["src"].as<int>();
    int dest = node["dest"].as<int>();
    bool reliable = node["reliable"].as<bool>();
    int size = node["size"].as<int>();

    or_protocol_msgs::Packet pkt;
    pkt.header.msg_type = or_protocol_msgs::Header::PAYLOAD;
    pkt.header.src_id = src;
    pkt.header.dest_id = dest;
    pkt.header.reliable = reliable;

    std_msgs::UInt32 seq;
    seq.data = 0;

    flows[src][dest] = {pkt, seq, size};
  }
  size_t total_msgs = sample["total_msgs"].as<size_t>();

  struct sigaction siginthandler;
  siginthandler.sa_handler = signal_handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  std::string log_file = "or_node.bag";
  rosbag::Bag bag;
  bag.open(log_file, rosbag::bagmode::Write);
  if (!bag.isOpen()) {
    ROS_FATAL("[main] failed to open log file: %s", log_file.c_str());
    exit(EXIT_FAILURE);
  } else {
    ROS_INFO("[main] logging received messages to %s", log_file.c_str());
  }

  or_protocol::ORProtocol or_node(argv[1]);
  or_protocol::ORProtocolTest or_node_test;
  or_node_test.initializeORNode(or_node, sample_link_etx);
  or_node.register_recv_func(msg_cb);

  ROS_INFO("[main] sending %ld messages", total_msgs);
  while (run && or_node.is_running()) {
    for (auto& src : flows) {
      for (auto& dest : src.second) {
        FlowSpec& flow = dest.second;

        flow.packet.header.hops = 0;
        flow.packet.data.clear();

        or_protocol::pack_msg(flow.packet, flow.seq);
        const int pad_bytes = flow.total_size - serializationLength(flow.packet);
        flow.packet.data.insert(flow.packet.data.end(), pad_bytes, 1);

        or_node.send(flow.packet);
        {
          std::lock_guard<std::mutex> lock(bag_mutex);
          bag.write("send_seq_numbers", ros::Time::now(), flow.seq);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        flow.seq.data++;
      }
    }
  }

  ROS_INFO("[main] sleeping for 4 seconds to allow all messages to transmit");
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  ROS_INFO("[main] send test complete");

  return 0;
}
