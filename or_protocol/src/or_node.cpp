#include <algorithm>
#include <filesystem>
#include <iostream>

#include <or_protocol/constants.h>
#include <or_protocol/or_node.h>
#include <or_protocol/utils.h>
#include <ros/serialization.h>

#include <or_protocol_msgs/Log.h>
#include <or_protocol_msgs/Packet.h>
#include <std_msgs/UInt32.h>


namespace or_protocol {


using or_protocol_msgs::Log;


ORNode::ORNode(std::string _IP, int _port)
{
  // TODO use interface name instead? enabling automatic IP address fetching

  OR_INFO("starting ORNode @ %s:%d", _IP.c_str(), _port);

  std::string id_str = _IP.substr(_IP.find_last_of('.') + 1);
  node_id = std::stoi(id_str);

  // initialize BCastSocket and register recv function handle
  namespace ph = std::placeholders;
  buff_recv_func fcn = std::bind(&ORNode::recv, this, ph::_1, ph::_2);
  bcast_socket.reset(new BCastSocket(_IP, _port, fcn));

  run = bcast_socket->is_running();  // true if bcast_socket initialized successfully

  ros::Time::init(); // enables ros::Time without an attached ROS node, master

  // set up log file
  std::filesystem::path log_dir;
  char* home_dir_c_str = std::getenv("HOME");
  if (home_dir_c_str) {
    log_dir = std::filesystem::path(home_dir_c_str) / ".ros";
    if (!std::filesystem::exists(log_dir)) {
      OR_WARN("%s does not exist", log_dir.c_str());
      log_dir = std::filesystem::path(".");
    }
  } else {
    OR_WARN("unable to fetch HOME environment variable");
    log_dir = std::filesystem::path(".");
  }
  std::filesystem::path log_file = log_dir / "or-protocol.bag";
  bag.open(log_file.string(), rosbag::bagmode::Write);
  if (!bag.isOpen()) {
    OR_FATAL("failed to open log file: %s", log_file.c_str());
    run = false;
  } else {
    OR_INFO("logging to: %s", log_file.c_str());
  }

  // start main packet processing thread
  process_thread = std::thread(&ORNode::process_packets, this);
}


ORNode::~ORNode()
{
  run = false;
  process_thread.join();
  bag.close();
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
    case or_protocol_msgs::Header::ACK:
      return std::string("ACK");
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
  std::string rel_str = header.reliable ? ", REL" : "";
  std::string type_str = packet_type_string(header);
  OR_DEBUG("%s: [%d] %d > %d via %d, %d %s, seq=%d, try=%d, %s%s",
           msg.c_str(), node_id, header.src_id, header.dest_id, header.curr_id,
           size, data_type.c_str(), header.seq, header.attempt, type_str.c_str(),
           rel_str.c_str());
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
  msg.header.attempt = 0;

  // manually serialize message so that we can keep a copy of buffer_ptr around
  // for later retransmission of reliable messages (i.e. reimplement
  // ros::serialization::serializMessage but around buffer_ptr instead of
  // SerializedMessage)
  // TODO check if message needs to be chunked
  uint32_t len = ros::serialization::serializationLength(msg) + 4;
  buffer_ptr buff_ptr(new char[len]);
  ros::serialization::OStream s(reinterpret_cast<uint8_t*>(buff_ptr.get()), len);
  ros::serialization::serialize(s, len - 4);
  ros::serialization::serialize(s, msg);

  if (send(buff_ptr.get(), len)) {
    ros::Time now = ros::Time::now();
    print_msg_info("send", msg.header, len);
    log_message(msg.header, Log::SEND, len, now);

    // queue message for re-transmission (will get cancelled if an ACK is
    // received within RETRY_DELAY)
    if (msg.header.reliable) {
      {
        std::lock_guard<std::mutex> lock(retrans_mutex);
        retransmission_set.emplace(msg.header.seq);
      }

      PacketQueueItemPtr item(new PacketQueueItem(buff_ptr, len, msg.header, now));
      item->send_time = now + ros::Duration(0, RETRY_DELAY);
      item->processed = true;
      item->retransmission = true;
      item->retries = MAX_RETRY_COUNT;

      push_packet_queue(item);
    }
    return true;
  }
  return false;
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

  PacketQueueItemPtr item(new PacketQueueItem(buff_ptr, size, header, now));
  push_packet_queue(item);

  print_msg_info("recv", header, size);
  log_message(header, Log::RECEIVE, size, now);
}


// TODO move logging to a separate thread?
void ORNode::log_message(const or_protocol_msgs::Header& header,
                         const int action,
                         const int size,
                         const ros::Time& time)
{
  static const std::string topic = "node" + std::to_string(node_id);
  Log msg;
  msg.header = header;
  msg.action = action;
  msg.size = size;
  std::lock_guard<std::mutex> lock(log_mutex);
  bag.write(topic, time, msg);
}


void ORNode::process_packets()
{
  // TODO reduce time spent busy waiting?
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

    // TODO check packet queue for overflow?

    //
    // re-queue processing
    //

    // in order to approximate desired opportunistic relaying priority, some
    // packets may be processed but held for a short period of time before
    // being re-transmitted
    if (item->processed) {
      // determine if processed message should be dropped, delayed, or sent
      // TODO determine a smarter way of requeuing so that the packet is not
      // delayed unnecessarily (step through queue and insert packet before
      // others with more recent send_time?)
      if (item->send_time > ros::Time::now()) {
        push_packet_queue(item);
      } else if (item->retransmission) {
        retrans_mutex.lock();
        if (retransmission_set.count(item->header.seq) > 0) {
          retrans_mutex.unlock();

          item->header.attempt++;
          update_msg_header(item->buffer(), item->header);
          send(item->buffer(), item->size);

          ros::Time now = ros::Time::now();
          double actual_dt = (now - item->recv_time).toSec() * 1000;
          double target_dt = (item->send_time - item->recv_time).toSec() * 1000;
          char buff[33];
          snprintf(buff, 33, "retry (t=%.2fms, a=%.2fms)", target_dt, actual_dt);
          print_msg_info(buff, item->header, item->size);
          log_message(item->header, Log::RETRY, item->size, now);

          // add message back to queue if retries remain
          item->retries--;
          if (item->retries > 0) {
            item->send_time += ros::Duration(0, RETRY_DELAY);
            push_packet_queue(item);
          }
        } else {
          retrans_mutex.unlock();
          print_msg_info("canceled retry", item->header, item->size);
        }
      } else if (item->priority < msg_priority(item->header)) {
        send(item->buffer(), item->size);
        ros::Time now = ros::Time::now();
        double actual_dt = (now - item->recv_time).toSec() * 1000;
        double target_dt = (item->send_time - item->recv_time).toSec() * 1000;
        char buff[30];
        snprintf(buff, 30, "relay (t=%.2fms, a=%.2fms)", target_dt, actual_dt);
        print_msg_info(buff, item->header, item->size);
        log_message(item->header, Log::RELAY, item->size, now);
      } else {
        char buff[6];
        snprintf(buff, 6, "%d > %d", item->priority, msg_priority(item->header));
        print_msg_info("superseded (drop)", item->header, item->size);
      }
      continue;
    }

    //
    // new packet processing
    //

    // TODO update statistics: node_states[header.curr_id].update_stats(header);

    // filter out messages that originated from the current node (i.e. echoed
    // back from an intermediate relay)
    if (item->header.src_id == node_id) {
      print_msg_info("echo (drop)", item->header, item->size);
      continue;
    }

    // update received messages queue (and highest priority msg received so far)
    // and filter out any that have already been received/processed
    bool new_msg = node_states[item->header.src_id].update_queue(item->header);
    if (!new_msg) {
      print_msg_info("dupe (drop)", item->header, item->size);
      continue;
    }

    // relay message
    if (item->header.dest_id != node_id) {

      // update ACKed message in node_states so that no more relays are sent
      if (item->header.msg_type == or_protocol_msgs::Header::ACK) {
        uint32_t ack_seq = extract_ack(item);
        // the source of the original message is the destination of the ACK
        node_states[item->header.dest_id].ack_msg(ack_seq);
      }

      unsigned int sender_priority = relay_priority(item->header.curr_id, item->header);
      unsigned int current_priority = relay_priority(node_id, item->header);

      if (current_priority == item->header.relays.size()) {
        print_msg_info("not relay (drop)", item->header, item->size);
      } else if (current_priority > sender_priority) {
        print_msg_info("low priority (drop)", item->header, item->size);
      } else {

        // update current transmitting node and packet hop count
        item->header.curr_id = node_id;
        item->header.hops++;
        update_msg_header(item->buffer(), item->header);

        // requeue relay with a delay to allow for the reception of ACKs
        // TODO if we expect the destination to receive the original
        // transmission, we should delay the relay to allow for ACKs to be
        // received to avoid unnecessary relays; however, if it is unlikely the
        // destination will receive the original transmission then we are adding
        // unnecessary latency and shouldn't wait to relay
        const ros::Duration delay(0, UNIT_DELAY * (current_priority + 1));
        item->send_time = item->recv_time + delay;
        item->priority = current_priority;
        item->processed = true;
        push_packet_queue(item);
        print_msg_info("queue", item->header, item->size);
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
      send(item->buffer(), item->size);
      print_msg_info("reply ping", item->header, item->size);
      log_message(item->header, Log::SEND, item->size, ros::Time::now());
      continue;
    }

    // process received acknowledgements
    if (item->header.msg_type == or_protocol_msgs::Header::ACK) {
      uint32_t ack_seq = extract_ack(item);
      {
        std::lock_guard<std::mutex> lock(retrans_mutex);
        if (retransmission_set.count(ack_seq) > 0)
          retransmission_set.erase(ack_seq);
      }
      print_msg_info("got ACK", item->header, item->size);
      continue;
    }

    // send acknowledgement, if required
    if (item->header.reliable) {
      or_protocol_msgs::Packet ack;

      std_msgs::UInt32 ack_seq;
      ack_seq.data = item->header.seq;
      pack_msg(ack, ack_seq);

      ack.header.msg_type = or_protocol_msgs::Header::ACK;
      ack.header.dest_id = item->header.src_id;
      ack.header.reliable = false;
      std::reverse_copy(item->header.relays.begin(),
                        item->header.relays.end(),
                        ack.header.relays.begin());
      send(ack);  // calls print_msg_info and log_message internally
    }

    if (recv_handle) {
      // deserialize entire message
      or_protocol_msgs::Packet msg;
      deserialize(msg, reinterpret_cast<uint8_t*>(item->buffer()), item->size);
      // TODO don't block receiving thread
      print_msg_info("deliver", item->header, item->size);
      recv_handle(item->recv_time, msg, node_id, item->size);
    }
  }
}


}  // namespace or_protocol
