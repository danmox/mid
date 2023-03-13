#include <filesystem>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <or_protocol/types.h>
#include <or_protocol/utils.h>
#include <or_protocol/network_state.h>

#include <ros/package.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt32.h>


using or_protocol::PacketQueueItem;


/* test header manipulation */
TEST(TestSuite, header_update)
{
  // original message

  or_protocol_msgs::Packet msg_in;
  msg_in.header.msg_type = or_protocol_msgs::Header::PAYLOAD;
  msg_in.header.src_id = 43;
  msg_in.header.curr_id = 142;
  msg_in.header.dest_id = 124;
  msg_in.header.seq = 2;
  msg_in.header.hops = 1;
  msg_in.header.relays[0] = 3;
  msg_in.header.relays[3] = 43;
  msg_in.data = std::vector<uint8_t>(5, 6);

  ros::SerializedMessage smsg_in = ros::serialization::serializeMessage(msg_in);

  char* msg_buff = reinterpret_cast<char*>(smsg_in.buf.get());
  size_t msg_size = smsg_in.num_bytes;

  // extract header for serialized original message, update extracted header and
  // original message, re-serialize

  or_protocol_msgs::Header header_in;
  or_protocol::deserialize(header_in, reinterpret_cast<uint8_t*>(msg_buff), msg_size);

  header_in.src_id = 25;
  header_in.curr_id = 24;
  header_in.relays[2] = 28;
  header_in.relays[3] = 29;

  msg_in.header.src_id = 25;
  msg_in.header.curr_id = 24;
  msg_in.header.relays[2] = 28;
  msg_in.header.relays[3] = 29;

  // update original message and compare

  or_protocol::update_msg_header(msg_buff, header_in);

  or_protocol_msgs::Packet msg_out;
  or_protocol::deserialize(msg_out, reinterpret_cast<uint8_t*>(msg_buff), msg_size);

  EXPECT_EQ(msg_in, msg_out);
}


/* test time serialization / de-serialization */
TEST(TestSuite, time_serialization)
{
  or_protocol_msgs::Packet msg_in;

  ros::Time::init();
  std_msgs::Time t_in;
  t_in.data = ros::Time::now();

  ros::SerializedMessage snow = ros::serialization::serializeMessage(t_in);
  msg_in.data.insert(msg_in.data.end(), snow.buf.get(), snow.buf.get() + snow.num_bytes);
  ros::SerializedMessage smsg = ros::serialization::serializeMessage(msg_in);

  char* buff = reinterpret_cast<char*>(smsg.buf.get());
  size_t size = smsg.num_bytes;

  or_protocol_msgs::Packet msg_out;
  or_protocol::deserialize(msg_out, reinterpret_cast<uint8_t*>(buff), size);

  std_msgs::Time t_out;
  or_protocol::deserialize(t_out, msg_out.data.data(), msg_out.data.size());

  EXPECT_EQ(msg_in, msg_out);
  EXPECT_EQ(t_in, t_out);
}


/* test ACK serialization / de-serialization */
TEST(TestSuite, ack_serialization)
{
  or_protocol_msgs::Packet msg_in;

  std_msgs::UInt32 seq_in;
  seq_in.data = 1523;

  ros::SerializedMessage sint = ros::serialization::serializeMessage(seq_in);
  msg_in.data.insert(msg_in.data.end(), sint.buf.get(), sint.buf.get() + sint.num_bytes);

  // manually serialize to get access to a buffer_ptr
  uint32_t msg_len = ros::serialization::serializationLength(msg_in) + 4;
  or_protocol::buffer_ptr buff_ptr(new char[msg_len]);
  ros::serialization::OStream s(reinterpret_cast<uint8_t*>(buff_ptr.get()), msg_len);
  ros::serialization::serialize(s, msg_len - 4);
  ros::serialization::serialize(s, msg_in);

  // create PacketQueueItem
  ros::Time dummy_time = ros::Time(0);
  or_protocol::PacketQueueItemPtr item(new PacketQueueItem(buff_ptr,
                                                           size_t(msg_len),
                                                           msg_in.header,
                                                           dummy_time));

  std_msgs::UInt32 seq_out;
  seq_out.data = or_protocol::extract_ack(item);

  EXPECT_EQ(seq_in, seq_out);
}


namespace fs = std::filesystem;


/* test automatic route selection */
TEST(TestSuite, route_selection)
{
  std::string package_path = ros::package::getPath("or_protocol");
  fs::path sample_dir = fs::path(package_path) / "test" / "samples";

  if (!fs::exists(sample_dir) || !fs::is_directory(sample_dir))
    ASSERT_TRUE(false) << "directory: " << sample_dir << " does not exist";

  std::vector<fs::path> sample_files;
  for (const auto& file : fs::directory_iterator(sample_dir))
    if (fs::is_regular_file(file) && file.path().extension() == ".yaml")
      sample_files.push_back(file.path());

  for (const fs::path& file : sample_files) {
    YAML::Node sample = YAML::LoadFile(file.string());

    or_protocol::ETXMap sample_link_etx;
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

    or_protocol::IntArrayMap route_relays;
    for (const auto& node : sample["relay_map"]) {
      const int& relay = node.first.as<int>();
      const std::vector<int>& relays_vec = node.second.as<std::vector<int>>();
      or_protocol::RelayArray relays = {0, 0, 0, 0};
      for (size_t i = 0; i < relays_vec.size(); ++i)
        relays[i] = relays_vec[i];
      route_relays.emplace(relay, relays);
    }

    uint8_t src = sample["src"].as<uint8_t>();
    uint8_t dest = sample["dest"].as<uint8_t>();
    for (const auto& item : route_relays) {
      or_protocol::NetworkState ns;
      ns.set_etx_map(sample_link_etx);
      ns.update_routes(item.first);
      or_protocol::FixedRoutingMap routing_map = ns.get_routing_map();
      EXPECT_EQ(routing_map[src][dest], route_relays[item.first]);
    }
  }
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
