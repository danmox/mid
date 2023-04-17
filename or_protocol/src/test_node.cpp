#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <or_protocol/or_protocol.h>
#include <or_protocol/utils.h>

#include <or_protocol_msgs/Packet.h>
#include <std_msgs/UInt32.h>


namespace fs = std::filesystem;
using ros::serialization::serializationLength;


volatile bool run = true;
rosbag::Bag bag;
std::mutex bag_mutex;
std::shared_ptr<or_protocol::ORProtocol> or_node;


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


void msg_cb(ros::Time recv_time, or_protocol_msgs::Packet& pkt, int node_id, int size)
{
  ROS_DEBUG("received message at %d with size %d", node_id, size);

  std_msgs::UInt32 seq;
  or_protocol::deserialize(seq, pkt.data.data(), pkt.data.size());

  std::lock_guard<std::mutex> lock(bag_mutex);
  or_node->log_ros_msg("recv_seq_numbers", recv_time, seq);
}


int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cout << "[main] usage: or_test <ip address> <sample.yaml>" << std::endl;
    return 0;
  }

  std::string ip_str(argv[1]);
  int node_id = std::stoi(ip_str.substr(ip_str.find_last_of('.') + 1));

  // load flows and etx table from yaml
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
  std::unordered_map<int, std::unordered_map<int, FlowSpec>> flows;
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

    flows[src][dest] = {pkt, seq, size};
  }
  int total_msgs = 0;
  if (flows.size() > 0 && sample["total_msgs"])
    total_msgs = sample["total_msgs"].as<int>();

  if (flows.size() > 0) {
    for (auto& src : flows) {
      for (auto& dest : src.second) {
        const FlowSpec& flow = dest.second;
        ROS_INFO("flow: %d > %d, size=%d, reliable=%s",
                 flow.packet.header.src_id, flow.packet.header.dest_id,
                 flow.total_size, flow.packet.header.reliable ? "true" : "false");
      }
    }
  } else {
    ROS_INFO("no flows originating at this node");
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = signal_handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  or_node.reset(new or_protocol::ORProtocol(argv[1]));
  or_node->register_recv_func(msg_cb);

  bag.open("test_node.bag", rosbag::bagmode::Write);

  if (total_msgs != 0)
    ROS_INFO("[main] sending %d messages", total_msgs);
  int sent_msgs = 0;
  while (run && or_node->is_running()) {
    for (auto& src : flows) {
      for (auto& dest : src.second) {
        FlowSpec& flow = dest.second;

        flow.packet.header.hops = 0;
        flow.packet.data.clear();

        or_protocol::pack_msg(flow.packet, flow.seq);
        const int pad_bytes = flow.total_size - serializationLength(flow.packet);
        flow.packet.data.insert(flow.packet.data.end(), pad_bytes, 1);

        or_node->send(flow.packet);
        {
          std::lock_guard<std::mutex> lock(bag_mutex);
          or_node->log_ros_msg("send_seq_numbers", ros::Time::now(), flow.seq);
        }
        flow.seq.data++;
      }
    }

    if (total_msgs != 0 && !(sent_msgs < total_msgs))
      break;

    ++sent_msgs;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ROS_INFO("[main] sleeping for 4 seconds to allow all messages to transmit");
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  ROS_INFO("[main] send test complete");

  return 0;
}
