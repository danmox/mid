#include <algorithm>
#include <filesystem>
#include <fmt/format.h>
#include <random>

#include <or_protocol/constants.h>
#include <or_protocol/or_protocol.h>
#include <or_protocol/utils.h>
#include <ros/serialization.h>

#include <or_protocol_msgs/Packet.h>
#include <std_msgs/UInt32.h>


namespace or_protocol {


using std::string;


ORProtocol::ORProtocol(string _IP)
{
  // TODO use interface name instead? enabling automatic IP address fetching

  OR_INFO("starting ORNode @ %s:%d", _IP.c_str(), OR_PROTOCOL_PORT);

  node_id = std::stoi(_IP.substr(_IP.find_last_of('.') + 1));

  // initialize BCastSocket and register recv function handle
  namespace ph = std::placeholders;
  buff_recv_func fcn = std::bind(&ORProtocol::recv, this, ph::_1, ph::_2);
  bcast_socket.reset(new BCastSocket(_IP, OR_PROTOCOL_PORT, fcn));

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
  std::filesystem::path log_filename = log_dir / "or-protocol.txt";
  log_file = fopen(log_filename.c_str(), "w");
  if (log_file == NULL) {
    OR_FATAL("failed to open log file: %s", log_filename.c_str());
    run = false;
  } else {
    OR_INFO("logging to: %s", log_filename.c_str());
  }

  // start worker threads
  process_thread = std::thread(&ORProtocol::process_packets, this);
  beacon_rx_thread = std::thread(&ORProtocol::process_beacons, this);
  beacon_tx_thread = std::thread(&ORProtocol::transmit_beacons, this);
  routing_thread = std::thread(&ORProtocol::compute_routes, this);
  log_thread = std::thread(&ORProtocol::process_log_queue, this);
}


ORProtocol::~ORProtocol()
{
  run = false;
  process_thread.join();
  beacon_rx_thread.join();
  beacon_tx_thread.join();
  routing_thread.join();
  log_thread.join();
  fclose(log_file);
}


// assuming node specific message header information has not been completed
bool ORProtocol::send(or_protocol_msgs::Packet& msg, const bool set_relays)
{
  msg.header.src_id = node_id;
  msg.header.curr_id = node_id;
  msg.header.seq = getSeqNum();
  msg.header.hops++;
  if (set_relays)
    msg.header.relays = network_state.relays(msg.header);

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
    log_event(msg.header, PacketAction::SEND, len, now);

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

      packet_queue.push(item);
    }
    return true;
  }
  return false;
}


bool ORProtocol::send(const char* buff, size_t size)
{
  if (!bcast_socket->send(buff, size)) {
    OR_ERROR("failed to send message");
    return false;
  }
  return true;
}


void ORProtocol::recv(buffer_ptr& buff_ptr, size_t size)
{
  ros::Time now = ros::Time::now();

  // deserialize just the header to determine future forwarding decisions
  or_protocol_msgs::Header header;
  deserialize(header, reinterpret_cast<uint8_t*>(buff_ptr.get()), size);

  PacketQueueItemPtr item(new PacketQueueItem(buff_ptr, size, header, now));
  packet_queue.push(item);

  log_event(header, PacketAction::RECEIVE, size, now);
}


void ORProtocol::log_event(const or_protocol_msgs::Header& header,
                           const PacketAction action,
                           const int size,
                           const ros::Time& time,
                           const string& msg)
{
  log_queue.push(std::make_tuple(header, action, size, time, msg));
}


void ORProtocol::log_event(const PacketQueueItemPtr& item,
                           const PacketAction action,
                           const ros::Time& time,
                           const std::string& msg)
{
  log_queue.push(std::make_tuple(item->header, action, item->size, time, msg));
}


void ORProtocol::process_packets()
{
  // TODO reduce time spent busy waiting?
  while (run) {

    int queue_size = packet_queue.size();
    if (queue_size > 1000)
      ROS_WARN_THROTTLE(0.1, "[ORNode] packet_queue.size() == %d!", queue_size);

    std::shared_ptr<PacketQueueItem> item;
    if (!packet_queue.pop(item))
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
        packet_queue.push(item);
      } else if (item->retransmission) {
        int seq_count;
        {
          std::lock_guard<std::mutex> lock(retrans_mutex);
          seq_count = retransmission_set.count(item->header.seq);
        }
        if (seq_count > 0) {
          item->header.attempt++;
          item->header.relays = network_state.relays(item->header);
          update_msg_header(item->buffer(), item->header);
          send(item->buffer(), item->size);

          ros::Time now = ros::Time::now();
          double actual_dt = (now - item->recv_time).toSec() * 1000;
          double target_dt = (item->send_time - item->recv_time).toSec() * 1000;
          string msg = fmt::format("t={:.2f}ms a={:.2f}ms", target_dt, actual_dt);
          log_event(item, PacketAction::RETRY, now, msg);

          // add message back to queue if retries remain
          item->retries--;
          if (item->retries > 0) {
            item->send_time += ros::Duration(0, RETRY_DELAY);
            packet_queue.push(item);
          }
        } else {
          log_event(item->header, PacketAction::CANCEL_RETRY, item->size,
                    ros::Time::now());
        }
      } else if (item->priority < msg_priority(item->header)) {
        send(item->buffer(), item->size);  // TODO update relays here too?
        ros::Time now = ros::Time::now();
        double actual_dt = (now - item->recv_time).toSec() * 1000;
        double target_dt = (item->send_time - item->recv_time).toSec() * 1000;
        string msg = fmt::format("t={:.2f}ms a={:.2f}ms", target_dt, actual_dt);
        log_event(item->header, PacketAction::RELAY, item->size, now, msg);
      } else {
        log_event(item, PacketAction::DROP_SUP, ros::Time::now());
      }
      continue;
    }

    //
    // new packet processing
    //

    // status messages are pushed onto a queue for processing by a separate
    // thread and are not routed / processed further in this thread
    if (item->header.msg_type == or_protocol_msgs::Header::STATUS) {
      network_state.push_beacon_queue(item);
      continue;
    }

    // filter out messages that originated from the current node (i.e. echoed
    // back from an intermediate relay)
    if (item->header.src_id == node_id) {
      log_event(item, PacketAction::DROP_ECHO, ros::Time::now());
      continue;
    }

    // update received messages queue (and highest priority msg received so far)
    // and filter out any that have already been received/processed
    MsgStatus ms = network_state.update_queue(item);
    if (!ms.is_new_msg) {
      log_event(item, PacketAction::DROP_DUP, ros::Time::now());
      continue;
    }

    // relay message
    if (item->header.dest_id != node_id) {

      // update ACKed message in network_state so that no more relays are sent
      if (item->header.msg_type == or_protocol_msgs::Header::ACK) {
        uint32_t ack_seq = extract_ack(item);
        // the source of the original message is the destination of the ACK
        network_state.ack_msg(item->header.dest_id, ack_seq, item->recv_time.toSec());
      }

      const unsigned int tx_priority = relay_priority(item->header.curr_id, item->header);
      const unsigned int rx_priority = relay_priority(node_id, item->header);

      if (rx_priority == item->header.relays.size()) {
        continue;
      } else if (rx_priority > tx_priority) {
        log_event(item, PacketAction::DROP_LOWPRI, ros::Time::now());
      } else {

        // update current transmitting node and packet hop count
        item->header.curr_id = node_id;
        item->header.hops++;
        item->header.relays = network_state.relays(item->header);
        update_msg_header(item->buffer(), item->header);

        // relay immediately or with some delay
        if (rx_priority == 0) {
          send(item->buffer(), item->size);
          log_event(item, PacketAction::RELAY, ros::Time::now());
        } else {
          const ros::Duration delay(0, UNIT_DELAY * rx_priority);
          item->send_time = item->recv_time + delay;
          item->priority = rx_priority;
          item->processed = true;
          packet_queue.push(item);
          log_event(item, PacketAction::QUEUE, ros::Time::now());
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
      item->header.relays = network_state.relays(item->header);
      update_msg_header(item->buffer(), item->header);
      send(item->buffer(), item->size);
      log_event(item, PacketAction::SEND, ros::Time::now());
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
      string msg = fmt::format("ack_seq={}", ack_seq);
      log_event(item, PacketAction::ACK, ros::Time::now(), msg);

      // don't ACK messages sent without routing (i.e. has a relay array with
      // all zeros; this signifies the message is meant as a one-time broadcast)
      // TODO kind of brittle? introduce a MINI_ACK for this purpose?
      if (item->header.relays[0] == 0)
        continue;
    }

    // if reliable, send full acknowledgement (with routing), otherwise send a
    // one-time ACK (will not get relayed) to terminate cooperative relaying
    // (this will prevent the highest priority relay from always transmitting
    // even if the destination receives the packet)
    or_protocol_msgs::Packet ack;
    std_msgs::UInt32 ack_seq;
    ack_seq.data = item->header.seq;
    pack_msg(ack, ack_seq);
    ack.header.msg_type = or_protocol_msgs::Header::ACK;
    ack.header.dest_id = item->header.src_id;
    ack.header.reliable = false;
    if (item->header.reliable) {
      send(ack);  // send ACK with routing
    } else {
      send(ack, false);  // send one-time ACK broadcast
    }

    if (ms.is_new_seq) {
      log_event(item, PacketAction::DELIVER, ros::Time::now());
      AppQueueItemPtr app_item(new AppQueueItem(item));
      app_queue.push(app_item);
    }
  }
}


void ORProtocol::process_beacons()
{
  while (run) {
    network_state.process_beacon_queue(node_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}


void ORProtocol::transmit_beacons()
{
  or_protocol_msgs::PacketPtr beacon(new or_protocol_msgs::Packet());
  beacon->header.msg_type = or_protocol_msgs::Header::STATUS;
  beacon->header.hops = 0;
  beacon->header.attempt = 0;
  beacon->header.reliable = false;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> jitter_dist(-BEACON_JITTER, BEACON_JITTER);

  unsigned int it = 1;
  ros::Time t0 = ros::Time::now();
  while (run) {
    beacon->header.hops = 0;
    beacon->data.clear();

    or_protocol_msgs::NetworkStatusPtr ptr = network_state.generate_beacon();
    pack_msg(*beacon, *ptr);

    const bool set_routes = false;
    send(*beacon, set_routes);

    // pass network status to application for logging purposes
    AppQueueItemPtr app_item(new AppQueueItem(beacon));
    app_queue.push(app_item);

    int offset_ms = it * BEACON_INTERVAL + jitter_dist(gen);
    ros::Duration offset = ros::Duration(offset_ms / 1000, (offset_ms % 1000) * 1e6);
    int sleep_ms = (t0 + offset - ros::Time::now()).toSec() * 1e3;

    if (sleep_ms > 0) {
      OR_DEBUG("beacon thread: sleeping for %dms", sleep_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    } else {
      OR_WARN("negative sleep_duration_ms (%dms) in transmit_beacons()!", sleep_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(int(BEACON_INTERVAL / 2.0)));
    }

    it++;
  }
}


void ORProtocol::compute_routes()
{
  or_protocol_msgs::PacketPtr pkt(new or_protocol_msgs::Packet());
  pkt->header.msg_type = or_protocol_msgs::Header::ROUTING_TABLE;
  pkt->header.curr_id = node_id;
  pkt->header.src_id = node_id;

  ros::Time target_time = ros::Time::now();
  while (run) {
    network_state.update_routes(node_id);

    // pass routing table to application for logging purposes
    pkt->data.clear();
    or_protocol_msgs::RoutingTablePtr s = network_state.get_routing_table_msg(node_id);
    pack_msg(*pkt, *s);
    AppQueueItemPtr app_item(new AppQueueItem(pkt));
    app_queue.push(app_item);

    target_time += ros::Duration(ROUTING_UPDATE_INTERVAL * 1e-3);
    int sleep_ms = (target_time - ros::Time::now()).toSec() * 1e3;
    if (sleep_ms > 0) {
      OR_DEBUG("routes thread: sleeping for %dms", sleep_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    } else {
      OR_WARN("negative sleep_duration_ms (%dms) in compute_routes()!", sleep_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(ROUTING_UPDATE_INTERVAL / 2));
    }
  }
}


void ORProtocol::process_log_queue()
{
  const string topic = "/node" + std::to_string(node_id) + "/log";

  while (run) {

    LogQueueItem item;
    while (log_queue.pop(item)) {
      const auto [hdr, action, size, time, msg] = item;
      const string rel_str = hdr.reliable ? "TRUE" : "FALSE";
      const string type_str = packet_type_string(hdr);

      std::lock_guard<std::mutex> lock(log_mutex);
      fmt::print(log_file, "[{:.9f}] {}: {} {}: {} > {} v {} bytes={} relays=[{}"
                 ", {}, {}, {}] seq={} att={} rel={} {}\n", time.toSec(), node_id,
                 packet_action_string(action).c_str(), type_str.c_str(), hdr.src_id,
                 hdr.dest_id, hdr.curr_id, size, hdr.relays[0], hdr.relays[1],
                 hdr.relays[2], hdr.relays[3], hdr.seq, hdr.attempt, rel_str.c_str(),
                 msg.c_str());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}


}  // namespace or_protocol
