#include <or_protocol/utils.h>

#include <frontier_exploration/Goal.h>
#include <frontier_exploration/FrontierGoals.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int64.h>


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


std::string packet_type_string(const or_protocol_msgs::Header& header)
{
  switch (header.msg_type) {
    case or_protocol_msgs::Header::STATUS:
      return std::string("STATUS");
    case or_protocol_msgs::Header::PAYLOAD:
      return std::string("PAYLOAD");
    case or_protocol_msgs::Header::PING_REQ:
      return std::string("PING_REQ");
    case or_protocol_msgs::Header::PING_RES:
      return std::string("PING_RES");
    case or_protocol_msgs::Header::ACK:
      return std::string("ACK");
    case or_protocol_msgs::Header::ROUTING_TABLE:
      return std::string("ROUTING_TABLE");
    case or_protocol_msgs::Header::TOPIC_INFO:
      return std::string("TOPIC_INFO");
    default:
      return std::string("UNKNOWN");
  }
}


std::string packet_action_string(const PacketAction action)
{
  switch (action) {
    case PacketAction::RECEIVE:
      return std::string("RECEIVE");
    case PacketAction::RELAY:
      return std::string("RELAY");
    case PacketAction::RETRY:
      return std::string("RETRY");
    case PacketAction::SEND:
      return std::string("SEND");
    case PacketAction::CANCEL_RETRY:
      return std::string("CANCEL_RETRY");
    case PacketAction::DROP_SUP:
      return std::string("DROP_SUP");
    case PacketAction::DROP_DUP:
      return std::string("DROP_DUP");
    case PacketAction::DROP_ECHO:
      return std::string("DROP_ECHO");
    case PacketAction::DROP_LOWPRI:
      return std::string("DROP_LOWPRI");
    case PacketAction::QUEUE:
      return std::string("QUEUE");
    case PacketAction::ACK:
      return std::string("ACK");
    case PacketAction::DELIVER:
      return std::string("DELIVER");
    default:
      return std::string("UNKNOWN");
  }
}


topic_tools::ShapeShifter get_msg_info(const std::string& type,
                                       const std::string& latch)
{
  topic_tools::ShapeShifter ss;

  if (type == "frontier_exploration/FrontierGoals")
    morph_shape_shifter<frontier_exploration::FrontierGoals>(ss, latch);
  else if (type == "frontier_exploration/Goal")
    morph_shape_shifter<frontier_exploration::Goal>(ss, latch);
  else if (type == "std_msgs/Int64")
    morph_shape_shifter<std_msgs::Int64>(ss, latch);
  else
    OR_ERROR("get_msg_info not implemented for type %s!", type.c_str());

  return ss;
}


} // namespace or_protocol
