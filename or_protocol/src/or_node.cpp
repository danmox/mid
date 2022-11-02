#include <chrono>
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

  run = true;
}


bool ORNode::send(const or_protocol_msgs::PacketConstPtr& msg)
{
  ros::SerializedMessage m = ros::serialization::serializeMessage(*msg);
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

  printf("recv: %d: %d > %d, seq: %d, %ld bytes\n", node_id, msg.src_id,
         msg.dest_id, msg.seq, msg.data.size());
}


void ORNode::send_loop(or_protocol_msgs::PacketPtr& msg) {
  msg->src_id = node_id;

  // TODO devise better way of shutting down
  while (run && bcast_socket->run) {
    send(msg);
    printf("send: %d: %d > %d, seq: %d, %ld bytes\n", node_id, msg->src_id,
           msg->dest_id, msg->seq, msg->data.size());
    msg->seq++;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
}


void ORNode::recv_loop()
{
  while (run && bcast_socket->run) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}


} // namespace or_protocol
