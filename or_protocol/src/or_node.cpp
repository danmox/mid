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

  run = bcast_socket->run;  // true if bcast_socket initializes successfully

  // start main packet processing thread
  process_thread = std::thread(&ORNode::process_packets, this);
}


ORNode::~ORNode()
{
  run = false;
  process_thread.join();
}


void ORNode::register_recv_func(msg_recv_func fcn)
{
  recv_handle = fcn;
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
    default:
      return std::string("UNKNOWN");
  }
}


void ORNode::print_msg_info(std::string msg,
                            const or_protocol_msgs::Header& header,
                            int size,
                            bool total)
{
  std::string data_type = total ? "bytes" : "data bytes";
  OR_DEBUG("%s: [%d] %d > %d via %d, %d %s, seq=%d, type=%s", msg.c_str(),
           node_id, header.src_id, header.dest_id, header.curr_id, size,
           data_type.c_str(), header.seq, packet_type_string(header).c_str());
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


// assuming node specific message header information has not been completed
bool ORNode::send(or_protocol_msgs::Packet& msg, bool fill_src)
{
  if (fill_src) {
    msg.header.src_id = node_id;
  }
  msg.header.curr_id = node_id;
  msg.header.seq = seq++;
  msg.header.hops++;

  print_msg_info("send", msg.header, msg.data.size(), false);

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


void ORNode::recv(buffer_ptr& buff_ptr, size_t size)
{
  std::lock_guard<std::mutex> queue_lock(queue_mutex);
  std::shared_ptr<PacketQueueItem> item(new PacketQueueItem(buff_ptr, size));
  packet_queue.push_back(item);
  OR_DEBUG("queued message");
}


void ORNode::process_packets()
{
  while (run) {

    std::shared_ptr<PacketQueueItem> item;
    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex);
      if (packet_queue.size() > 0) {
        item = packet_queue.front();
        packet_queue.pop_front();
      }
    }
    if (!item)
      continue;

    // in order to approximate desired opportunistic relaying priority, some
    // packets may be processed but held for a short period of time before
    // actually transmitted
    if (item->processed) {
      // requeue message if it should not be sent yet
      // TODO determine a smarter way of doing this so that the packet is not
      // delayed unnecessarily (step through queue and insert packet before
      // others with more recent min_transmit_time?)
      if (item->min_transmit_time > ros::Time::now()) {
        std::lock_guard<std::mutex> queue_lock(queue_mutex);
        packet_queue.push_back(item);
      } else {
        send(item->buffer(), item->size);
      }
      continue;
    }

    // process message

    // deserialize only header to determine forwarding decisions
    or_protocol_msgs::Header header;
    deserialize(header, reinterpret_cast<uint8_t*>(item->buffer()), item->size);

    print_msg_info("process", header, item->size, true);

    // TODO update statistics: node_states[header.curr_id].update_stats(header);

    // filter out messages that originated from the current node (i.e. echoed
    // back from an intermediate relay) or have already been received/processed
    bool new_msg = false;
    if (header.src_id != node_id) {
      new_msg = node_states[header.src_id].update_queue(header);
    }
    if (!new_msg) {
      print_msg_info("drop", header, item->size, true);
      continue;
    }

    // relay standard messages
    if (header.dest_id != node_id) {
      // TODO take into account forwarding preferences by appropriately setting
      // the min_transmit_time and requeueing the message if this node is not
      // the highest priority relay

      // update current transmitting node and packet hop count
      header.curr_id = node_id;
      header.hops++;
      update_msg_header(item->buffer(), header);
      print_msg_info("relay", header, item->size, true);
      send(item->buffer(), item->size);
      continue;
    }

    // respond to ping requests
    if (header.msg_type == or_protocol_msgs::Header::PING_REQ) {
      header.msg_type = or_protocol_msgs::Header::PING_RES;
      header.dest_id = header.src_id;
      header.curr_id = node_id;
      header.src_id = node_id;
      header.hops++;
      update_msg_header(item->buffer(), header);
      print_msg_info("reply ping", header, item->size, true);
      send(item->buffer(), item->size);
      continue;
    }

    if (recv_handle) {
      // deserialize entire message
      or_protocol_msgs::Packet msg;
      deserialize(msg, reinterpret_cast<uint8_t*>(item->buffer()), item->size);
      // TODO don't block receiving thread?
      print_msg_info("deliver", header, item->size, true);
      recv_handle(msg, node_id, item->size);
    }
  }
}


}  // namespace or_protocol
