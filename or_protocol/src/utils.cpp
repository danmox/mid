#include <or_protocol/utils.h>
#include <std_msgs/UInt32.h>


namespace or_protocol {


int relay_priority(int id, const or_protocol_msgs::Header& header)
{
  size_t priority = 0;
  while (priority < header.relays.size() && header.relays[priority] != id)
    priority++;

  return priority;
}


uint32_t extract_ack(const PacketQueueItemPtr& ptr)
{
  std_msgs::UInt32 ack_seq;
  const int header_size = ros::serialization::serializationLength(ptr->header);
  // the ACK's payload is the sequence number of the original message
  deserialize(ack_seq,
              reinterpret_cast<uint8_t*>(ptr->buffer() + header_size + 8),
              ptr->size - header_size - 8);  // should be 8 bytes
  return ack_seq.data;
}


void update_msg_header(char* buff, const or_protocol_msgs::Header& header)
{
  // prepends the total size of the message as an uint32_t
  ros::SerializedMessage sheader = ros::serialization::serializeMessage(header);
  // NOTE buff contains: msg size (4 bytes), serialized header (M bytes), ...
  // sheader contains: header size (4 bytes), serialized header (M bytes)
  // we overwrite the bytes associated with msg.header with the bytes of header
  memcpy(buff + 4, sheader.buf.get() + 4, sheader.num_bytes - 4);
}


} // namespace or_protocol
