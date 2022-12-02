#include <iostream>

#include <or_protocol/or_node.h>
#include <ros/serialization.h>

#include <or_protocol/utils.h>
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

  run = bcast_socket->is_running();  // true if bcast_socket initialized successfully

  // start main packet processing thread
  process_thread = std::thread(&ORNode::process_packets, this);

  ros::Time::init();
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


void ORNode::print_msg_info(const std::string& msg,
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
  msg.header.seq = getSeqNum();
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
  ros::Time now = ros::Time::now();

  // deserialize just the header to determine future forwarding decisions
  or_protocol_msgs::Header header;
  deserialize(header, reinterpret_cast<uint8_t*>(buff_ptr.get()), size);

  std::lock_guard<std::mutex> queue_lock(queue_mutex);
  PacketQueueItemPtr item(new PacketQueueItem(buff_ptr, size, header, now));
  packet_queue.push_back(item);

  print_msg_info("recv", header, size);
}


void ORNode::process_packets()
{
  // TODO reduce busy waiting?
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
    // being re-transmitted
    if (item->processed) {
      // determine if processed message should be dropped, delayed, or sent
      // TODO determine a smarter way of requeuing so that the packet is not
      // delayed unnecessarily (step through queue and insert packet before
      // others with more recent send_time?)
      if (item->send_time > ros::Time::now()) {
        std::lock_guard<std::mutex> queue_lock(queue_mutex);
        packet_queue.push_back(item);
      } else if (item->priority < msg_priority(item->header)) {
        send(item->buffer(), item->size);
        double actual_dt = (ros::Time::now() - item->recv_time).toSec() * 1000;
        double target_dt = (item->send_time - item->recv_time).toSec() * 1000;
        char buff[30];
        snprintf(buff, 30, "relay (t=%.2fms, a=%.2fms)", target_dt, actual_dt);
        print_msg_info(buff, item->header, item->size);
      } else {
        char buff[6];
        snprintf(buff, 6, "%d > %d", item->priority, msg_priority(item->header));
        print_msg_info("superseded (drop)", item->header, item->size);
      }
      continue;
    }

    // TODO update statistics: node_states[header.curr_id].update_stats(header);

    // filter out messages that originated from the current node (i.e. echoed
    // back from an intermediate relay)
    if (item->header.src_id == node_id) {
      print_msg_info("echo (drop)", item->header, item->size);
      continue;
    }

    // update received messages queue (and highest priority msg received so far)
    // and filter out any that have already been received/processed
    if (!node_states[item->header.src_id].update_queue(item->header)) {
      print_msg_info("dupe (drop)", item->header, item->size);
      continue;
    }

    // relay message
    if (item->header.dest_id != node_id) {

      int sender_priority = relay_priority(item->header.curr_id, item->header);
      int current_priority = relay_priority(node_id, item->header);

      if (current_priority == item->header.relays.size()) {
        print_msg_info("not relay (drop)", item->header, item->size);
      } else if (current_priority > sender_priority) {
        print_msg_info("low priority (drop)", item->header, item->size);
      } else {

        // update current transmitting node and packet hop count
        item->header.curr_id = node_id;
        item->header.hops++;
        update_msg_header(item->buffer(), item->header);

        // relay message or requeue if it should be delayed
        if (current_priority == 0) {
          send(item->buffer(), item->size);
          print_msg_info("relay", item->header, item->size);
        } else {
          const ros::Duration delay(0, UNIT_DELAY * current_priority);
          item->send_time = item->recv_time + delay;
          item->priority = current_priority;
          item->processed = true;
          std::lock_guard<std::mutex> queue_lock(queue_mutex);
          packet_queue.push_back(item);
          print_msg_info("queue", item->header, item->size);
        }
      }
      continue;
    }

    // respond to ping requests
    if (item->header.msg_type == or_protocol_msgs::Header::PING_REQ) {
      item->header.msg_type = or_protocol_msgs::Header::PING_RES;
      item->header.dest_id = item->header.src_id;
      item->header.curr_id = node_id;
      item->header.src_id = node_id;
      item->header.seq = getSeqNum();
      item->header.hops++;
      update_msg_header(item->buffer(), item->header);
      print_msg_info("reply ping", item->header, item->size);
      send(item->buffer(), item->size);
      continue;
    }

    if (recv_handle) {
      // deserialize entire message
      or_protocol_msgs::Packet msg;
      deserialize(msg, reinterpret_cast<uint8_t*>(item->buffer()), item->size);
      // TODO don't block receiving thread
      print_msg_info("deliver", item->header, item->size);
      recv_handle(msg, node_id, item->size);
    }
  }
}


}  // namespace or_protocol
