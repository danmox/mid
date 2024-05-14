#include <ros/ros.h>
#include <unordered_map>

#include <or_protocol_msgs/NetworkStatus.h>


using or_protocol_msgs::NetworkStatus;
using or_protocol_msgs::ETXEntry;


ros::Publisher status_pub;


void status_cb(const NetworkStatus::ConstPtr& msg)
{
  static std::unordered_map<int, or_protocol_msgs::Point> positions;
  static std::unordered_map<int, std::unordered_map<int, ETXEntry>> etx_table;

  NetworkStatus status;

  for (const or_protocol_msgs::Point& pt : msg->positions) {
    if (positions[pt.node].seq == 0 || positions[pt.node].seq < pt.seq) {
      positions[pt.node] = pt;
    }
    status.positions.push_back(positions[pt.node]);
  }

  for (const or_protocol_msgs::ETXList& list : msg->etx_table) {
    or_protocol_msgs::ETXList new_list;
    new_list.node = list.node;
    for (const or_protocol_msgs::ETXEntry& entry : list.etx_list) {
      if (etx_table[list.node][entry.node].seq == 0 ||
          etx_table[list.node][entry.node].seq < entry.seq) {
        etx_table[list.node][entry.node] = entry;
      }
      new_list.etx_list.push_back(etx_table[list.node][entry.node]);
    }
    status.etx_table.push_back(new_list);
  }

  status_pub.publish(status);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "status_aggregator");

  ros::NodeHandle nh;

  status_pub = nh.advertise<NetworkStatus>("agg_status", 5);
  ros::Subscriber status_sub = nh.subscribe("status", 5, status_cb);

  ros::spin();

  return 0;
}
