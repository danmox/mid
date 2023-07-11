#include <or_protocol/or_node.h>
#include <ros/xmlrpc_manager.h>

#include <or_protocol_msgs/TopicInfo.h>
#include <std_msgs/UInt16.h>


namespace or_protocol {


using XmlRpc::XmlRpcValue;


ORNode::ORNode(const ros::NodeHandle& _nh, const ros::NodeHandle& _pnh) :
  run(false), nh(_nh), pnh(_pnh)
{
  std::string ip;
  if (!pnh.getParam("IP", ip)) {
    ORN_FATAL("unable to fetch required param 'IP'");
    return;
  }

  XmlRpc::XmlRpcValue topics_param;
  if (!pnh.getParam("sync_topics", topics_param)) {
    ORN_FATAL("unable to fetch required param 'sync_topics'");
    return;
  }

  if (topics_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ORN_FATAL("'sync_topics' not an instance of XmlRpcValue::TypeArray; is it formatted correctly?");
    return;
  }

  status_pub = nh.advertise<or_protocol_msgs::NetworkStatus>("network_status", 1);
  table_pub = nh.advertise<or_protocol_msgs::RoutingTable>("routing_table", 1);

  namespace ph = std::placeholders;

  if (topics_param.size() > 0) {
    try {
      for (const auto& t : topics_param[0]) {
        std::string topic(t.first);
        int queue_size = 1;
        int dest_id_i;
        bool reliable = false;

        if (!get_param(t.second, XmlRpcValue::TypeBoolean, "reliable", reliable))
          ORN_WARN("topic %s: using default value for reliable of '%s'", topic.c_str(), reliable ? "TRUE": "FALSE");
        if (!get_param(t.second, XmlRpcValue::TypeInt, "queue_size", queue_size))
          ORN_WARN("topic %s: using default value for queue_size of %d", topic.c_str(), queue_size);
        if (!get_param(t.second, XmlRpcValue::TypeInt, "dest_id", dest_id_i)) {
          ORN_ERROR("topic %s: unable to setup topic without valid dest_id", topic.c_str());
          continue;
        }
        unsigned int dest_id = dest_id_i;

        int idx = subscribers.size();
        auto cb = std::bind(&ORNode::ros_msg_cb, this, ph::_1, idx);
        ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(topic, queue_size, cb);
        std::unique_ptr<ros::Subscriber> ptr(new ros::Subscriber(sub));

        subscribers.push_back(SubInfo{topic, queue_size, reliable, false, dest_id, std::move(ptr)});
        ORN_INFO("Adding local subscriber: %s: {queue_size: %d, reliable: %s, dest_id: %d}", topic.c_str(), queue_size, reliable ? "true" : "false", dest_id);

        // TODO publishers
      }
    } catch (const XmlRpc::XmlRpcException& e) {
      ORN_FATAL("failure while parsing 'sync_topic' param: %s", e.getMessage().c_str());
      return;
    }
  } else {
    ORN_WARN("no local subscribers configured");
  }

  msg_recv_func cb = std::bind(&ORNode::recv_packet, this, ph::_1, ph::_2, ph::_3);
  protocol.reset(new ORProtocol(ip, cb));

  run = protocol->is_running();
}


void ORNode::ros_msg_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int idx)
{
  SubInfo& subscriber = subscribers[idx];

  or_protocol_msgs::Packet pkt;
  pkt.header.msg_type = or_protocol_msgs::Header::PAYLOAD;
  pkt.header.dest_id = subscriber.dest_id;  // TODO multicast?
  pkt.header.reliable = subscriber.reliable;

  std_msgs::UInt16 id_msg;
  id_msg.data = idx;
  pack_msg(pkt, id_msg);

  or_protocol::pack_msg(pkt, *msg);

  ORN_INFO("sending packet to %d", pkt.header.dest_id);
  protocol->send(pkt);
}


void ORNode::recv_packet(ros::Time& recv_time, or_protocol_msgs::Packet& pkt, int size)
{
  ORN_DEBUG("[%f] received pkt from %d via %d with %d bytes", recv_time.toSec(), pkt.header.src_id, pkt.header.curr_id, size);

  uint8_t* buff = pkt.data.data();
  const int buff_size = pkt.data.size();

  const auto& type = pkt.header.msg_type;
  if (type == or_protocol_msgs::Header::ACK) {
    return;

  } else if (type == or_protocol_msgs::Header::TOPIC_INFO) {
    or_protocol_msgs::TopicInfo info;
    deserialize(info, buff, buff_size);

    PubKey key = std::make_pair(pkt.header.src_id, info.id);
    auto it = publishers.find(key);
    if (it != publishers.end()) {
      if (info.md5sum != it->second.md5sum || info.type != it->second.type) {
        ORN_ERROR("Conflicting TopicInfo received for node %d for topic '%s': new {type: %s, md5sum: %s}, existing {type: %s, md5sum: %s}", pkt.header.src_id, info.name.c_str(), info.type.c_str(), info.md5sum.c_str(), it->second.type.c_str(), it->second.md5sum.c_str());
      } else {
        ORN_DEBUG("Already registered TopicInfo from node %d: {name: %s, type: %s}", pkt.header.src_id, info.name.c_str(), info.type.c_str());
      }
    } else {
      ORN_INFO("Registering new TopicInfo from node %d: {name: %s, type: %s, md5sum: %s, latching: %s}", pkt.header.src_id, info.name.c_str(), info.type.c_str(), info.md5sum.c_str(), info.latching.c_str());

      topic_tools::ShapeShifter ss;
      ss.morph(info.md5sum, info.type, info.definition, info.latching);

      // TODO queue size parameter
      std::unique_ptr<ros::Publisher> pub(new ros::Publisher(ss.advertise(nh, info.name, 1)));
      publishers[key] = PubInfo{info.name, info.md5sum, info.type, info.definition, info.latching, std::move(pub)};
    }

  } else if (type == or_protocol_msgs::Header::STATUS) {
    or_protocol_msgs::NetworkStatus status_msg;
    deserialize(status_msg, buff, buff_size);
    status_pub.publish(status_msg);

  } else if (type == or_protocol_msgs::Header::ROUTING_TABLE) {
    or_protocol_msgs::RoutingTable table_msg;
    deserialize(table_msg, buff, buff_size);
    table_pub.publish(table_msg);

  } else if (type == or_protocol_msgs::Header::PAYLOAD) {
    std_msgs::UInt16 id;
    deserialize(id, buff, buff_size);

    PubKey key = std::make_pair(pkt.header.src_id, id.data);
    auto it = publishers.find(key);
    if (it == publishers.end()) {
      ORN_WARN("no PubInfo found for key {node: %d, id: %d}, dropping!", pkt.header.src_id, id.data);
    } else {
      PubInfo& info = it->second;
      topic_tools::ShapeShifter ss;
      ss.morph(info.md5sum, info.type, info.definition, info.latching);
      // offset to first msg byte: 4 (UInt16 msg size) + 2 (UInt16 msg) + 4 (ShapeShifter msg size)
      ros::serialization::IStream is(buff + 10, buff_size - 10);
      try {
        ros::serialization::Serializer<topic_tools::ShapeShifter>::read(is, ss);
      } catch (const ros::Exception& e) {
        ORN_ERROR("Error reading PAYLOAD into ShapeShifter: %s", e.what());
      }

      ORN_DEBUG("publishing PAYLOAD from %d on %s", pkt.header.src_id, info.name.c_str());
      info.pub->publish(ss);
    }

  } else {
    ORN_DEBUG("unknown message type '%s'", packet_type_string(pkt.header).c_str());
  }
}


}  // namespace or_protocol


int main(int argc, char** argv)
{
  ros::init(argc, argv, "or_node");
  ros::NodeHandle nh, pnh("~");
  or_protocol::ORNode or_node(nh, pnh);

  if (or_node.running())
    ros::spin();

  return 0;
}
