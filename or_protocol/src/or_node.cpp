#include <iostream>

#include <or_protocol/or_node.h>
#include <ros/serialization.h>

#include <or_protocol_msgs/Packet.h>


namespace or_protocol {


ORNode::ORNode(std::string _IP, int _port)
{
  // TODO use interface name instead? enabling automatic IP address fetching

  std::string id_str = _IP.substr(_IP.find_last_of('.') + 1);
  node_id = std::stoi(id_str);

  // initialize BCastSocket and register recv function handle
  using namespace std::placeholders;
  buff_recv_func fcn = std::bind(&ORNode::recv, this, _1, _2);
  bcast_socket.reset(new BCastSocket(_IP, _port, fcn));
}


void ORNode::register_recv_func(msg_recv_func fcn)
{
  recv_handle = fcn;
}


std::string packet_type_string(const or_protocol_msgs::Packet& msg)
{
  switch (msg.header.msg_type) {
    case or_protocol_msgs::Header::STATUS:
      return std::string("STATUS");
    case or_protocol_msgs::Header::PAYLOAD:
      return std::string("PAYLOAD");
    case or_protocol_msgs::Header::PING_REQ:
      return std::string("PING_REQ");
    case or_protocol_msgs::Header::PING_RES:
      return std::string("PING_RES");
    default:
      return std::string("UNKNOWN");
  }
}

void ORNode::print_msg_info(std::string msg,
                            const or_protocol_msgs::Packet& packet) {
  OR_DEBUG("%s: [%d] %d > %d via %d, %ld bytes, seq=%d, type=%s", msg.c_str(),
           node_id, packet.header.src_id, packet.header.dest_id,
           packet.header.curr_id, packet.data.size(), packet.header.seq,
           packet_type_string(packet).c_str());
}


bool ORNode::send(const or_protocol_msgs::PacketConstPtr& msg)
{
  ros::SerializedMessage m = ros::serialization::serializeMessage(*msg);
  return send(reinterpret_cast<const char*>(m.buf.get()), m.num_bytes);
}


// assuming node specific message header information has not been completed
bool ORNode::send(or_protocol_msgs::Packet& msg, bool fill_src)
{
  if (fill_src) {
    msg.header.src_id = node_id;
  }
  msg.header.curr_id = node_id;
  msg.header.seq = seq++;
  msg.header.hops++;

  print_msg_info("send", msg);

  ros::SerializedMessage m = ros::serialization::serializeMessage(msg);
  return send(reinterpret_cast<char*>(m.buf.get()), m.num_bytes);
}


bool ORNode::send(const char* buff, size_t size)
{
  if (!bcast_socket->send(buff, size)) {
    OR_ERROR("failed to send message");
    return false;
  }
  return true;
}


void ORNode::recv(char* buff, size_t size)
{
  or_protocol_msgs::Packet msg;
  deserialize(msg, reinterpret_cast<uint8_t*>(buff), size);

  print_msg_info("recv", msg);

  // relay standard messages
  if (msg.header.dest_id != node_id && msg.header.src_id != node_id) {
    // TODO keep track of which messages have been re-transmitted
    // TODO take into account forwarding preferences
    print_msg_info("relay", msg);
    send(msg, false); // don't overwrite src field when relaying
    return;
  }

  // respond to ping requests
  if (msg.header.msg_type == or_protocol_msgs::Header::PING_REQ) {
    msg.header.msg_type = or_protocol_msgs::Header::PING_RES;
    msg.header.dest_id = msg.header.src_id;
    send(msg);
    return;
  }

  if (recv_handle)
    recv_handle(msg, node_id, size);
}


} // namespace or_protocol
