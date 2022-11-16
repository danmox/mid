#include <gtest/gtest.h>

#include <or_protocol/or_node.h>
#include <or_protocol_msgs/Header.h>
#include <or_protocol_msgs/Packet.h>

#include <std_msgs/Time.h>


TEST(TestSuite, ping)
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


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
