#ifndef OR_PROTOCOL_OR_NODE_H_
#define OR_PROTOCOL_OR_NODE_H_

#include <unordered_map>
#include <utility>
#include <vector>

#include <or_protocol/or_protocol.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <or_protocol_msgs/Packet.h>


namespace or_protocol {


#define ORN_DEBUG(fmt, ...) ROS_DEBUG("[ORNode] " fmt, ##__VA_ARGS__)
#define ORN_INFO(fmt, ...) ROS_INFO("[ORNode] " fmt, ##__VA_ARGS__)
#define ORN_WARN(fmt, ...) ROS_WARN("[ORNode] " fmt, ##__VA_ARGS__)
#define ORN_ERROR(fmt, ...) ROS_ERROR("[ORNode] " fmt, ##__VA_ARGS__)
#define ORN_FATAL(fmt, ...) ROS_FATAL("[ORNode] " fmt, ##__VA_ARGS__)


struct SubInfo
{
  std::string topic;
  int queue_size;
  bool reliable;
  std::unique_ptr<ros::Subscriber> sub;
};


struct PubInfo
{
  std::string name;
  std::string md5sum;
  std::string type;
  std::string definition;
  std::string latching;
  std::unique_ptr<ros::Publisher> pub;
};


template <typename T>
bool get_param(const XmlRpc::XmlRpcValue& val,
               const XmlRpc::XmlRpcValue::Type type,
               std::string name,
               T& out_param)
{
  static std::unordered_map<XmlRpc::XmlRpcValue::Type, std::string>
    type_strings = {
      {XmlRpc::XmlRpcValue::TypeInvalid, "TypeInvalid"},
      {XmlRpc::XmlRpcValue::TypeBoolean, "TypeBoolean"},
      {XmlRpc::XmlRpcValue::TypeInt, "TypeInt"},
      {XmlRpc::XmlRpcValue::TypeDouble, "TypeDouble"},
      {XmlRpc::XmlRpcValue::TypeString, "TypeString"},
      {XmlRpc::XmlRpcValue::TypeDateTime, "TypeDateTime"},
      {XmlRpc::XmlRpcValue::TypeBase64, "TypeBase64"},
      {XmlRpc::XmlRpcValue::TypeArray, "TypeArray"},
      {XmlRpc::XmlRpcValue::TypeStruct, "TypeStruct"}};

  std::string val_str = val.toXml();
  const char* vcstr = val_str.c_str();
  if (!val.hasMember(name)) {
    ORN_ERROR("val '%s' does not have member '%s'", vcstr, name.c_str());
    return false;
  }
  if (!val[name].valid()) {
    ORN_ERROR("val '%s' is not valid", vcstr);
    return false;
  }
  if (val[name].getType() != type) {
    ORN_ERROR("val '%s' is not type '%s'", vcstr, type_strings[type].c_str());
    return false;
  }

  out_param = static_cast<T>(val[name]);
  return true;
}


class ORNode
{
 public:
  ORNode(const ros::NodeHandle& _nh, const ros::NodeHandle& _pnh);

  void recv_packet(ros::Time& recv_time, or_protocol_msgs::Packet& pkt, int size);

  void ros_msg_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int idx);

  bool running() { return run; }

 private:
  volatile bool run;

  ros::NodeHandle nh, pnh;
  ros::Publisher status_pub, table_pub;

  std::shared_ptr<ORProtocol> protocol;

  std::unordered_map<int, SubInfo> subscribers;
  std::unordered_map<int, PubInfo> publishers;

  std::vector<int> dest_ids;
};


}  // namespace or_protocol


#endif
