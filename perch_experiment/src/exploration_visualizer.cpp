#include <regex>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <exploration_msgs/FrontierGoals.h>
#include <exploration_msgs/Goal.h>
#include <geometry_msgs/Point.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <visualization_msgs/Marker.h>


using exploration_msgs::FrontierGoals;
using exploration_msgs::Goal;
using or_protocol_msgs::NetworkStatus;
using visualization_msgs::Marker;


geometry_msgs::Point to_point(const exploration_msgs::Point2D& p, const double z)
{
  geometry_msgs::Point point;
  point.x = p.x;
  point.y = p.y;
  point.z = z;
  return point;
}


Marker create_marker(const std::string& ns, const int id, const unsigned int type)
{
  Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time(0);  // display in rviz regardless
  marker.ns = ns;
  marker.id = id;
  marker.action = Marker::ADD;
  marker.type = type;
  marker.pose.orientation.w = 1.0;
  return marker;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_visualizer");

  ros::NodeHandle nh;
  ros::Publisher vis_pub = nh.advertise<Marker>("exploration", 5);

  std::regex goal_topic_re("^/([a-z]+)([0-9]+)/goals$");
  std::regex frontier_topic_re("^/([a-z]+)([0-9]+)/frontiers$");

  std::unordered_map<int, ros::Subscriber> goal_sub_map;
  std::unordered_map<int, ros::Subscriber> frontier_sub_map;
  std::unordered_map<int, exploration_msgs::Point2D> goal_map;
  std::unordered_map<int, std::vector<exploration_msgs::Point2D>> frontier_map;
  std::vector<exploration_msgs::Point2D> banned;

  std::unordered_map<int, exploration_msgs::Point2D> pos_map;
  auto status_cb = [&](const NetworkStatus::ConstPtr& msg) {
    for (const auto& p : msg->positions) {
      exploration_msgs::Point2D pt;
      pt.x = p.x;
      pt.y = p.y;
      pos_map[p.node] = pt;
    }
  };
  ros::Subscriber status_sub = nh.subscribe<NetworkStatus>("status", 5, status_cb);

  ros::Time last_update(0);
  int update_count = 0;

  ros::Rate loop_rate(10);
  while (ros::ok()) {

    ros::spinOnce();

    // dynamically subscribe to all /*/goals /*/frontiers topics

    if ((ros::Time::now() - last_update).toSec() > 2.0 && update_count < 4) {

      ros::master::V_TopicInfo topic_infos;
      ros::master::getTopics(topic_infos);

      if (topic_infos.size() == 0) {
        ROS_WARN("[exploration_vis] topics list is empty!");
      } else {

        // exploration_msgs/Goal subscribers
        for (const ros::master::TopicInfo& info : topic_infos) {

          std::smatch topic_match;
          if (!std::regex_match(info.name, topic_match, goal_topic_re))
            continue;

          if (topic_match.size() != 3)
            continue;

          int id = std::stoi(topic_match[2].str());
          if (goal_sub_map.count(id) > 0)
            continue;

          auto cb = [&, id](const Goal::ConstPtr& msg) {
            goal_map[id] = msg->goal;
          };
          goal_sub_map[id] = nh.subscribe<Goal>(info.name, 4, cb);
          ROS_INFO("[exploration_vis] created new exploration_msgs/Goal subscriber: {%d: %s}", id, info.name.c_str());
        }

        // exploration_msgs/FrontierGoals subscribers
        for (const ros::master::TopicInfo& info : topic_infos) {

          std::smatch topic_match;
          if (!std::regex_match(info.name, topic_match, frontier_topic_re))
            continue;

          if (topic_match.size() != 3)
            continue;

          int id = std::stoi(topic_match[2].str());
          if (frontier_sub_map.count(id) > 0)
            continue;

          auto cb = [&, id](const FrontierGoals::ConstPtr& msg) {
            if (msg->action == FrontierGoals::ADD)
              frontier_map[id] = msg->goals;
            else if (msg->action == FrontierGoals::SKIP)
              banned.insert(banned.begin(), msg->goals.begin(), msg->goals.end());
            else
              ROS_ERROR("[exploration_vis] invalid action in FrontierGoals message");
          };
          frontier_sub_map[id] = nh.subscribe<FrontierGoals>(info.name, 4, cb);
          ROS_INFO("[exploration_vis] created new exploration_msgs/FrontierGoals subscriber: {%d: %s}", id, info.name.c_str());
        }
      }

      update_count++;
    }

    // visualizations

    // goals

    Marker goal = create_marker("goals", 0, Marker::POINTS);
    goal.scale.x = 0.6;
    goal.scale.y = 0.6;
    goal.color.g = 1.0;
    goal.color.a = 1.0;
    for (const auto& pair : goal_map)
      goal.points.push_back(to_point(pair.second, 0.01));

    if (goal.points.size() > 0)
      vis_pub.publish(goal);

    // positions

    Marker pos = create_marker("pos", 0, Marker::SPHERE_LIST);
    pos.scale.x = 0.5;
    pos.scale.y = 0.5;
    pos.scale.z = 0.5;
    pos.color.r = 0.5;
    pos.color.g = 0.5;
    pos.color.b = 0.5;
    pos.color.a = 1.0;

    Marker lines = create_marker("lines", 0, Marker::LINE_LIST);
    lines.scale.x = 0.2;
    lines.color.r = 0.2;
    lines.color.g = 0.2;
    lines.color.b = 0.2;
    lines.color.a = 1.0;

    for (const auto& pair : pos_map) {
      geometry_msgs::Point pt = to_point(pair.second, 0.25);
      pos.points.push_back(pt);

      if (goal_map.count(pair.first) != 0) {
        lines.points.push_back(pt);
        lines.points.push_back(to_point(goal_map[pair.first], 0.0));
      }

      Marker text = create_marker("text", pair.first, Marker::TEXT_VIEW_FACING);
      text.scale.z = 1.0;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = std::to_string(pair.first);
      text.pose.position = pt;
      text.pose.position.z = 0.75;

      vis_pub.publish(text);
    }

    if (pos.points.size() > 0)
      vis_pub.publish(pos);
    if (lines.points.size() > 0)
      vis_pub.publish(lines);

    // frontiers

    Marker frontiers = create_marker("frontiers", 0, Marker::POINTS);
    frontiers.scale.x = 0.4;
    frontiers.scale.y = 0.4;
    frontiers.color.b = 1.0;
    frontiers.color.a = 1.0;
    for (const auto& pair : frontier_map)
      for (const auto& pt : pair.second)
        frontiers.points.push_back(to_point(pt, 0.02));

    if (frontiers.points.size() > 0)
      vis_pub.publish(frontiers);

    Marker banned_marker = create_marker("banned", 0, Marker::POINTS);
    banned_marker.scale.x = 0.2;
    banned_marker.scale.y = 0.2;
    banned_marker.color.r = 1.0;
    banned_marker.color.a = 1.0;
    for (const auto& pt : banned)
      banned_marker.points.push_back(to_point(pt, 0.03));

    if (banned_marker.points.size() > 0)
      vis_pub.publish(banned_marker);

    loop_rate.sleep();
  }

  return 0;
}
