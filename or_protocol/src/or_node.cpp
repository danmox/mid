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

  int node_id = std::stoi(ip.substr(ip.find_last_of('.') + 1));

  XmlRpc::XmlRpcValue task_agent_ids_param;
  if (!nh.getParam("/task_agent_ids", task_agent_ids_param)) {
    ORN_FATAL("failed to fetch required param '/task_agent_ids'");
    return;
  } else {
    for (int i = 0; i < task_agent_ids_param.size(); i++)
      if ((int)task_agent_ids_param[i] != node_id)
        dest_ids.push_back(task_agent_ids_param[i]);
  }
  std::string dest_id_str;
  for (int d : dest_ids)
    dest_id_str += " " + std::to_string(d);
  ORN_INFO("destination ids:%s", dest_id_str.c_str());

  XmlRpc::XmlRpcValue topics_param;
  if (!pnh.getParam("sync_topics", topics_param)) {
    ORN_FATAL("unable to fetch required param 'sync_topics'");
    return;
  }

  if (topics_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ORN_FATAL("'sync_topics' not an instance of XmlRpcValue::TypeArray; is it formatted correctly?");
    return;
  }

  status_pub = nh.advertise<or_protocol_msgs::NetworkStatus>("network_status", 5);
  table_pub = nh.advertise<or_protocol_msgs::RoutingTable>("routing_table", 5);

  namespace ph = std::placeholders;

  if (topics_param.size() > 0) {
    try {
      for (int array_idx = 0; array_idx < topics_param.size(); array_idx++) {
        for (auto& t : topics_param[array_idx]) {

          // NOTE topics_param[0] entries should be in the following format:
          // - /topic/name: {type: string, reliable: bool, queue_size: int, source: int}

          std::string topic(t.first);

          int queue_size = 1;
          bool reliable = false;
          std::string latch = "false";  // TODO implement and make parameter

          std::string type;
          int source;
          if (!get_param(t.second, XmlRpcValue::TypeString, "type", type)) {
            ORN_ERROR("topic %s: unable to capture traffic without valid 'type'", topic.c_str());
            continue;
          }
          if (!get_param(t.second, XmlRpcValue::TypeInt, "source", source)) {
            ORN_ERROR("topic %s: must contain source node", topic.c_str());
            continue;
          }
          if (!get_param(t.second, XmlRpcValue::TypeInt, "queue_size", queue_size))
            ORN_WARN("topic %s: using default value for 'queue_size' of %d", topic.c_str(), queue_size);
          if (!get_param(t.second, XmlRpcValue::TypeBoolean, "reliable", reliable))
            ORN_WARN("topic %s: using default value for 'reliable' of '%s'", topic.c_str(), reliable ? "TRUE": "FALSE");

          // only subscribe to topics we don't publish and vice versa to prevent
          // infinite message loops
          if (source == node_id) {
            auto cb = std::bind(&ORNode::ros_msg_cb, this, ph::_1, array_idx);
            ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(topic, queue_size, cb);
            std::unique_ptr<ros::Subscriber> ptr(new ros::Subscriber(sub));
            subscribers[array_idx] = SubInfo{topic, queue_size, reliable, std::move(ptr)};
            ORN_INFO("adding subscriber: %s: {type: %s, queue_size: %d, reliable: %s}", topic.c_str(), type.c_str(), queue_size, reliable ? "true" : "false");
          } else {
            topic_tools::ShapeShifter ss = get_msg_info(type, latch);
            std::unique_ptr<ros::Publisher> pub(new ros::Publisher(ss.advertise(nh, topic, queue_size)));
            publishers[array_idx] = PubInfo{topic, ss.getMD5Sum(), ss.getDataType(), ss.getMessageDefinition(), latch, std::move(pub)};
            ORN_INFO("adding publisher: %s: {type: %s, queue_size: %d, reliable: %s}", topic.c_str(), type.c_str(), queue_size, reliable ? "true" : "false");
          }
        }
      }
    } catch (const XmlRpc::XmlRpcException& e) {
      ORN_FATAL("failure while parsing 'sync_topic' param: %s", e.getMessage().c_str());
      return;
    }
  } else {
    ORN_WARN("no subscribers or publishers configured");
  }

  msg_recv_func cb = std::bind(&ORNode::recv_packet, this, ph::_1, ph::_2, ph::_3);
  protocol.reset(new ORProtocol(ip, cb));

  // after protocol is initialized to get access to ORProtocol::update_pose
  pose_sub = nh.subscribe("pose", 5, &ORProtocol::update_pose, protocol.get());

  run = protocol->is_running();
}


void ORNode::ros_msg_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int idx)
{
  if (subscribers.count(idx) == 0) {
    ORN_WARN("no SubInfo found for key %d; dropping!", idx);
    return;
  }
  SubInfo& subscriber = subscribers[idx];
  ORN_DEBUG("received ROS message on topic %s", subscriber.topic.c_str());

  or_protocol_msgs::Packet pkt;
  pkt.header.msg_type = or_protocol_msgs::Header::PAYLOAD;
  pkt.header.reliable = subscriber.reliable;

  std_msgs::UInt16 id_msg;
  id_msg.data = idx;
  pack_msg(pkt, id_msg);

  or_protocol::pack_msg(pkt, *msg);

  for (const int& dest : dest_ids) {
    ORN_DEBUG("sending packet to %d", dest);
    pkt.header.hops = 0;
    pkt.header.dest_id = dest;
    protocol->send(pkt);
  }
}


void ORNode::recv_packet(ros::Time& recv_time, or_protocol_msgs::Packet& pkt, int size)
{
  ORN_DEBUG("[%f] received pkt from %d via %d, type=%s, %d bytes", recv_time.toSec(), pkt.header.src_id, pkt.header.curr_id, packet_type_string(pkt.header).c_str(), size);

  uint8_t* buff = pkt.data.data();
  const int buff_size = pkt.data.size();

  const auto& type = pkt.header.msg_type;
  if (type == or_protocol_msgs::Header::ACK) {
    return;

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

    if (publishers.count((int)id.data) == 0) {
      ORN_WARN("no PubInfo found for key %d; dropping!", (int)id.data);
    } else {
      PubInfo& info = publishers[(int)id.data];
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
