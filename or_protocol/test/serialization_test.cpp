#include <gtest/gtest.h>

#include <or_protocol/or_node.h>
#include <or_protocol_msgs/Header.h>
#include <or_protocol_msgs/Packet.h>

#include <std_msgs/Time.h>
#include <std_msgs/UInt32.h>


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
  ros::SerializedMessage smsg = ros::serialization::serializeMessage(msg_in);

  char* buff = reinterpret_cast<char*>(smsg.buf.get());
  size_t size = smsg.num_bytes;

  std_msgs::UInt32 seq_out;
  uint32_t header_size = ros::serialization::serializationLength(msg_in.header);
  // msg size: 4 bytes msg size + header size + 4 bytes payload size + payload
  uint8_t* payload_ptr = reinterpret_cast<uint8_t*>(buff + header_size + 8);
  or_protocol::deserialize(seq_out, payload_ptr, size - header_size - 8);

  EXPECT_EQ(seq_in, seq_out);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
