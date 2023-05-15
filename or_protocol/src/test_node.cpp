#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <yaml-cpp/yaml.h>

//#include <rosbag/bag.h>
#include <or_protocol/or_protocol.h>
#include <or_protocol/utils.h>

#include <or_protocol_msgs/Packet.h>
#include <std_msgs/UInt32.h>


namespace fs = std::filesystem;
using ros::serialization::serializationLength;


std::atomic<bool> run = true;
std::shared_ptr<or_protocol::ORProtocol> or_node;
std::thread recv_thread;
//rosbag::Bag bag;
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


void msg_cb()
{
  while (run) {

    or_protocol::AppQueueItemPtr item;
    while (or_node->recv_msg(item)) {

      or_protocol_msgs::Packet pkt;
      int size;
      if (item->queue_ptr) {
        or_protocol::deserialize(pkt,
                                 reinterpret_cast<uint8_t*>(item->queue_ptr->buffer()),
                                 item->queue_ptr->size);
        size = item->queue_ptr->size;
      } else if (item->packet_ptr) {
        pkt = *item->packet_ptr;
        size = serializationLength(pkt);
      } else {
        ROS_ERROR("invalid queue_ptr and packet_ptr");
        continue;
      }

      ROS_DEBUG("received message with size %d", size);

      if (pkt.header.msg_type == or_protocol_msgs::Header::ROUTING_TABLE) {
        or_protocol_msgs::RoutingTable table;
        or_protocol::deserialize(table, pkt.data.data(), pkt.data.size());
        ROS_DEBUG("writing routing table to bag");
        //std::lock_guard<std::mutex> lock(bag_mutex);
        //bag.write(topic_prefix + "routes", recv_time, table);
      } else if (pkt.header.msg_type == or_protocol_msgs::Header::STATUS) {
        or_protocol_msgs::NetworkStatus status;
        or_protocol::deserialize(status, pkt.data.data(), pkt.data.size());
        ROS_DEBUG("writing status message to bag");
        //std::lock_guard<std::mutex> lock(bag_mutex);
        //bag.write(topic_prefix + "status", recv_time, status);
      } else if (pkt.header.msg_type == or_protocol_msgs::Header::PAYLOAD) {
        std_msgs::UInt32 seq;
        or_protocol::deserialize(seq, pkt.data.data(), pkt.data.size());
        ROS_DEBUG("writing received seq number to bag");
        //std::lock_guard<std::mutex> lock(bag_mutex);
        //bag.write(topic_prefix + "recv_seq_numbers", recv_time, seq);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  double rate = 10;
  if (sample["send_rate"])
    rate = sample["send_rate"].as<double>();
  int sleep_ms = 1.0 / rate * 1000;
  int delay_seconds = 20;
  if (sample["delay_seconds"])
    delay_seconds = sample["delay_seconds"].as<int>();

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

  //
  // run test
  //

  struct sigaction siginthandler;
  siginthandler.sa_handler = signal_handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  //bag.open("test_node.bag", rosbag::bagmode::Write);

  or_node.reset(new or_protocol::ORProtocol(argv[1]));
  topic_prefix = std::string("/node")
    + std::to_string(or_node->get_node_id()) + std::string("/");
  recv_thread = std::thread(msg_cb);

  ROS_INFO("[main] sleeping for %d seconds", delay_seconds);
  std::this_thread::sleep_for(std::chrono::seconds(delay_seconds));
  ROS_INFO("[main] starting main loop");

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
          //std::lock_guard<std::mutex> lock(bag_mutex);
          //bag.write(topic_prefix + "send_seq_numbers", ros::Time::now(), flow.seq);
        }
        flow.seq.data++;
      }
    }

    if (total_msgs != 0 && !(sent_msgs < total_msgs))
      break;

    ++sent_msgs;

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
  ROS_INFO("[main] done sending messages, awaiting shutdown signal");

  recv_thread.join();

  return 0;
}
