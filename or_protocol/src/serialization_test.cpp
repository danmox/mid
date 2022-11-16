#include <or_protocol/or_node.h>
#include <or_protocol_msgs/Header.h>
#include <or_protocol_msgs/Packet.h>


int main()
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

  char* msg_buff = reinterpret_cast<char *>(smsg_in.buf.get());
  size_t msg_size = smsg_in.num_bytes;

  std::vector<uint8_t> msg_buff_int(msg_buff, msg_buff + msg_size);

  // extract header for serialized original message, update, re-serialize

  or_protocol_msgs::Header header_in;
  or_protocol::deserialize(header_in, reinterpret_cast<uint8_t *>(msg_buff), msg_size);

  header_in.src_id = 25;
  header_in.curr_id = 24;
  header_in.relays[2] = 28;
  header_in.relays[4] = 29;

  msg_in.header.src_id = 25;
  msg_in.header.curr_id = 24;
  msg_in.header.relays[2] = 28;
  msg_in.header.relays[4] = 29;

  or_protocol_msgs::Header header_out;
  ros::SerializedMessage sheader_out = ros::serialization::serializeMessage(header_in);

  char* header_buff = reinterpret_cast<char*>(sheader_out.buf.get());
  size_t header_size = sheader_out.num_bytes;

  std::vector<uint8_t> header_buff_int(header_buff, header_buff + header_size);

  // update original message and compare

  or_protocol::update_msg_header(msg_buff, header_in);

  or_protocol_msgs::Packet msg_out;
  or_protocol::deserialize(msg_out, reinterpret_cast<uint8_t*>(msg_buff), msg_size);

  return 0;
}
