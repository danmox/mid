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
  recv_function fcn = std::bind(&ORNode::recv, this, _1, _2);
  bcast_socket.reset(new BCastSocket(_IP, _port, fcn));
}


void ORNode::print_msg_info(std::string msg,
                            const or_protocol_msgs::Packet& packet) {
  printf("%s: %d: %d > %d via %d, seq: %d, %ld bytes\n", msg.c_str(), node_id,
         packet.src_id, packet.dest_id, packet.curr_id, packet.seq,
         packet.data.size());
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
    msg.src_id = node_id;
  }
  msg.curr_id = node_id;
  msg.seq = seq++;

  print_msg_info("send", msg);

  ros::SerializedMessage m = ros::serialization::serializeMessage(msg);
  return send(reinterpret_cast<const char*>(m.buf.get()), m.num_bytes);
}


bool ORNode::send(const char* buff, size_t size)
{
  if (!bcast_socket->send(buff, size)) {
    std::fprintf(stderr, "[ORNode] failed to send message");
    return false;
  }
  return true;
}


void ORNode::recv(char* buff, size_t size)
{
  or_protocol_msgs::Packet msg;
  // the size of the message is prepended as an int32_t when sent
  ros::serialization::IStream s(reinterpret_cast<uint8_t*>(buff + 4), size - 4);
  // TODO deserialize only enough to make a choice about forwarding?
  ros::serialization::deserialize(s, msg);

  print_msg_info("recv", msg);

  // relay
  if (msg.dest_id != node_id &&
      msg.src_id != node_id &&
      msg.curr_id != node_id) {
    // TODO keep track of which messages have been re-transmitted
    print_msg_info("relay", msg);
    send(msg, false); // don't overwrite src field when relaying
  }
}


} // namespace or_protocol
