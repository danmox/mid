#include <regex>
#include <unordered_map>
#include <vector>

#include <or_protocol/network_state.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <or_protocol_msgs/RoutingTable.h>


using geometry_msgs::PoseStamped;


struct Pose2DStamped
{
    ros::Time stamp;
    double x;
    double y;
};


double compute_etx(const Pose2DStamped& p1, const Pose2DStamped& p2, double max_range)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dist = sqrt(dx * dx + dy * dy);
  double prob = std::max(1.0 - dist / max_range, 0.0);
  return prob < 1.0 / or_protocol::ETX_MAX ? or_protocol::ETX_MAX : 1.0 / prob;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_routing_node");

  ros::NodeHandle nh;

  ros::Publisher status_pub = nh.advertise<or_protocol_msgs::NetworkStatus>("network_state", 1);

  double max_range;
  if (!nh.getParam("/max_range", max_range)) {
    ROS_FATAL("[sim_routing] unable to fetch param '/max_range'");
    return EXIT_FAILURE;
  }

  ros::Time last_topic_update(0);
  int topic_update_count = 0;

  std::unordered_map<int, ros::Subscriber> sub_map;
  std::unordered_map<int, ros::Publisher> pub_map;
  std::unordered_map<int, Pose2DStamped> pose_map;
  std::vector<int> node_ids;

  std::regex topic_re("^/([a-z]+)([0-9]+)/pose$");

  ros::Rate loop_rate(2.0);
  while (ros::ok()) {

    ros::spinOnce();

    // subscribe to all /*/pose topics
    if ((ros::Time::now() - last_topic_update).toSec() > 2.0 && topic_update_count < 4) {
      ros::master::V_TopicInfo topic_infos;
      ros::master::getTopics(topic_infos);

      if (topic_infos.size() == 0) {
        ROS_WARN("[sim_routing] topics list is empty!");
      } else {
        std::smatch m;
        for (const ros::master::TopicInfo &info : topic_infos) {
          if (!std::regex_match(info.name, m, topic_re))
            continue;

          if (m.size() != 3)
            continue;

          int id = std::stoi(m[2].str());
          if (sub_map.count(id) > 0)
            continue;

          node_ids.push_back(id);

          auto cb = [=,&pose_map](const PoseStamped::ConstPtr& p)
          {
            pose_map[id] = Pose2DStamped{p->header.stamp, p->pose.position.x, p->pose.position.y};
          };
          sub_map[id] = nh.subscribe<PoseStamped>(info.name, 1, cb);
          ROS_INFO("[sim_routing] created new pose subscriber: {%d: %s}", id, info.name.c_str());

          std::string name = m[1].str() + m[2].str() + std::string("/routing_table");
          pub_map[id] = nh.advertise<or_protocol_msgs::RoutingTable>(name, 1);
        }
      }

      topic_update_count++;
    }

    if (node_ids.size() > 1 && pose_map.size() == node_ids.size()) {

      // compute ETX map
      or_protocol::ETXMap etx_map;
      for (const int src : node_ids) {
        std::unordered_map<int, double> inner_map;
        for (const int dst : node_ids) {
          if (src == dst)
            continue;
          double etx = compute_etx(pose_map[src], pose_map[dst], max_range);
          inner_map.emplace(dst, etx);
        }
        etx_map.emplace(src, inner_map);
      }

      or_protocol::NetworkState ns;
      ns.set_etx_map(etx_map);

      // send status message
      or_protocol_msgs::NetworkStatusPtr status_ptr = ns.generate_beacon();
      status_ptr->positions.clear();
      for (const auto& item : pose_map) {
        or_protocol_msgs::Point pt;
        pt.node = item.first;
        pt.x = item.second.x;
        pt.y = item.second.y;
        status_ptr->positions.push_back(pt);
      }
      status_pub.publish(status_ptr);

      // compute routes for each node
      for (const int id : node_ids) {
        ns.update_routes(id);
        or_protocol_msgs::RoutingTablePtr rt_ptr = ns.get_routing_table_msg();
        pub_map[id].publish(rt_ptr);
      }

    } else {
      ROS_WARN("[sim_routing] not enough agents/poses to compute routes");
    }

    loop_rate.sleep();
  }

  return 0;
}
