#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <yaml-cpp/yaml.h>

#include <or_protocol/or_protocol.h>
#include <or_protocol/utils.h>
#include <rosbag/bag.h>

#include <or_protocol_msgs/Packet.h>
#include <std_msgs/UInt32.h>


namespace fs = std::filesystem;
using ros::serialization::serializationLength;


std::atomic<bool> run = true, reply = false;
std::shared_ptr<or_protocol::ORProtocol> or_node;
std::thread recv_thread;
rosbag::Bag bag;
std::mutex bag_mutex;
std::string topic_prefix;


struct FlowSpec
{
  or_protocol_msgs::Packet packet;
  std_msgs::UInt32 seq;
  int total_size;
};


void signal_handler(int s)
{
  ROS_DEBUG("[main] received shutdown signal %d", s);
  run = false;
}


void pkt_cb(ros::Time& recv_time, or_protocol_msgs::Packet& pkt, int size)
{
  ROS_DEBUG("received message with size %d", size);

  if (pkt.header.msg_type == or_protocol_msgs::Header::ROUTING_TABLE) {
    or_protocol_msgs::RoutingTable table;
    or_protocol::deserialize(table, pkt.data.data(), pkt.data.size());
    ROS_DEBUG("writing routing table to bag");
    std::lock_guard<std::mutex> lock(bag_mutex);
    bag.write(topic_prefix + "routes", recv_time, table);
  } else if (pkt.header.msg_type == or_protocol_msgs::Header::STATUS) {
    or_protocol_msgs::NetworkStatus status;
    or_protocol::deserialize(status, pkt.data.data(), pkt.data.size());
    ROS_DEBUG("writing status message to bag");
    std::lock_guard<std::mutex> lock(bag_mutex);
    bag.write(topic_prefix + "status", recv_time, status);
  } else if (pkt.header.msg_type == or_protocol_msgs::Header::PAYLOAD) {
    if (reply) {
      pkt.header.dest_id = pkt.header.src_id;
      or_node->send(pkt);
    } else {
      std_msgs::UInt32 seq;
      or_protocol::deserialize(seq, pkt.data.data(), pkt.data.size());
      ROS_DEBUG("writing received seq number to bag");
      std::lock_guard<std::mutex> lock(bag_mutex);
      bag.write(topic_prefix + "recv_seq_numbers", recv_time, seq);
    }
  }
}


int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cout << "[main] usage: or_test <ip address> <sample.yaml>" << std::endl;
    return 0;
  }

  std::string ip_str(argv[1]);
  int node_id = std::stoi(ip_str.substr(ip_str.find_last_of('.') + 1));

  //
  // load flows and etx table from yaml
  //

  fs::path sample_path(argv[2]);
  if (!fs::is_regular_file(sample_path)) {
    std::cout << "[main] " << sample_path << " is not a file" << std::endl;
    return 0;
  }
  YAML::Node sample = YAML::LoadFile(sample_path.string());
  if (!sample["flows"]) {
    std::cout << "[main] 'flows' must be specified" << std::endl;
    return 0;
  }
  std::unordered_map<int, FlowSpec> flows;
  for (const auto& node : sample["flows"]) {
    if (!node["src"] || !node["dest"] || !node["reliable"] || !node["size"]) {
      std::cout << "[main] each flow must contain 'src', 'dest', 'reliable', and";
      std::cout << " 'size' fields" << std::endl;
      return 0;
    }
    int src = node["src"].as<int>();
    int dest = node["dest"].as<int>();
    bool reliable = node["reliable"].as<bool>();
    int size = node["size"].as<int>();

    if (src != node_id)
      continue;

    or_protocol_msgs::Packet pkt;
    pkt.header.msg_type = or_protocol_msgs::Header::PAYLOAD;
    pkt.header.src_id = src;
    pkt.header.dest_id = dest;
    pkt.header.reliable = reliable;

    std_msgs::UInt32 seq;
    seq.data = 0;

    flows[dest] = {pkt, seq, size};
  }
  int total_msgs = 0;
  if (flows.size() > 0 && sample["total_msgs"])
    total_msgs = sample["total_msgs"].as<int>();
  double rate = 10;
  if (sample["send_rate"])
    rate = sample["send_rate"].as<double>();
  int sleep_ms = 1.0 / rate * 1000;
  int delay_seconds = 20;
  if (sample["delay_seconds"])
    delay_seconds = sample["delay_seconds"].as<int>();

  if (flows.size() > 0) {
    for (auto& dest : flows) {
      const FlowSpec& flow = dest.second;
      ROS_INFO("flow: %d > %d, size=%d, reliable=%s",
               flow.packet.header.src_id, flow.packet.header.dest_id,
               flow.total_size, flow.packet.header.reliable ? "true" : "false");
    }
  } else {
    ROS_INFO("no flows originating at this node");
    reply = true;  // reflect traffic back to sender for accurate delay stats
  }

  //
  // run test
  //

  struct sigaction siginthandler;
  siginthandler.sa_handler = signal_handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  bag.open("test_node.bag", rosbag::bagmode::Write);

  topic_prefix = std::string("/node") + std::to_string(node_id) + std::string("/");

  or_node.reset(new or_protocol::ORProtocol(argv[1], pkt_cb));

  ROS_INFO("[main] sleeping for %d seconds", delay_seconds);
  int steps = delay_seconds * 10;
  for (int i = 0; i < steps && run; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ROS_INFO("[main] starting main loop");

  if (total_msgs != 0)
    ROS_INFO("[main] sending %d messages", total_msgs);
  int sent_msgs = 0;
  while (run && or_node->is_running()) {
    for (auto& dest : flows) {
      FlowSpec& flow = dest.second;

      flow.packet.header.hops = 0;
      flow.packet.data.clear();

      or_protocol::pack_msg(flow.packet, flow.seq);
      const int pad_bytes = flow.total_size - serializationLength(flow.packet);
      flow.packet.data.insert(flow.packet.data.end(), pad_bytes, 1);

      or_node->send(flow.packet);
      ros::Time send_time = ros::Time::now();
      {
        std::lock_guard<std::mutex> lock(bag_mutex);
        bag.write(topic_prefix + "send_seq_numbers", send_time, flow.seq);
      }
      flow.seq.data++;
    }

    ++sent_msgs;

    if (total_msgs != 0 && !(sent_msgs < total_msgs))
      break;

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
  ROS_INFO("[main] sent %d messages", sent_msgs);
  ROS_INFO("[main] done sending messages, awaiting shutdown signal");

  while (run)
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

  bag.close();

  return 0;
}
