#include <regex>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <exploration_msgs/FrontierGoals.h>
#include <exploration_msgs/Goal.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <visualization_msgs/Marker.h>


using exploration_msgs::FrontierGoals;
using exploration_msgs::Goal;
using or_protocol_msgs::NetworkStatus;
using visualization_msgs::Marker;


geometry_msgs::Point set_z(const geometry_msgs::Point& p, const double z)
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
  ros::init(argc, argv, "results_visualizer");

  ros::NodeHandle nh;
  ros::Publisher vis_pub = nh.advertise<Marker>("/experiment_vis", 5);

  std::regex goal_topic_re("^/([a-z]+)([0-9]+)/goals$");
  std::regex pose_topic_re("^/([a-z]+)([0-9]+)/world_pose");

  std::unordered_map<int, ros::Subscriber> goal_sub_map;
  std::unordered_map<int, ros::Subscriber> pose_sub_map;
  std::unordered_map<int, geometry_msgs::Point> goal_map;
  std::unordered_map<int, geometry_msgs::Point> pose_map;

  ros::Time last_update(0);
  int update_count = 0, attempts = 2;

  ros::Rate loop_rate(10);
  while (ros::ok()) {

    ros::spinOnce();

    // dynamically subscribe to all /*/goals /*/world_pose topics

    if ((ros::Time::now() - last_update).toSec() > 1.0 && update_count < attempts) {


      ros::master::V_TopicInfo topic_infos;
      ros::master::getTopics(topic_infos);

      if (topic_infos.size() == 0) {
        ROS_WARN("[results_vis] topics list is empty!");
      } else {

        // goal subscribers
        for (const ros::master::TopicInfo& info : topic_infos) {

          std::smatch topic_match;
          if (!std::regex_match(info.name, topic_match, goal_topic_re) ||
              topic_match.size() != 3)
            continue;

          int id = std::stoi(topic_match[2].str());
          if (goal_sub_map.count(id) > 0)
            continue;

          auto cb = [&, id](const Goal::ConstPtr& msg) {
            goal_map[id].x = msg->goal.x;
            goal_map[id].y = msg->goal.y;
          };
          goal_sub_map[id] = nh.subscribe<Goal>(info.name, 4, cb);
          ROS_INFO("[results_vis] created new exploration_msgs/Goal subscriber: {%d: %s}", id, info.name.c_str());
        }

        // pose subscribers
        for (const ros::master::TopicInfo& info : topic_infos) {

          std::smatch topic_match;
          if (!std::regex_match(info.name, topic_match, pose_topic_re) ||
              topic_match.size() != 3)
            continue;

          int id = std::stoi(topic_match[2].str());
          if (pose_sub_map.count(id) > 0)
            continue;

          auto cb = [&, id](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            pose_map[id] = msg->pose.position;
          };
          pose_sub_map[id] = nh.subscribe<geometry_msgs::PoseStamped>(info.name, 4, cb);
          ROS_INFO("[results_vis] created new geometry_msgs/PoseStamped subscriber: {%d: %s}", id, info.name.c_str());
        }

      }

      update_count++;
      last_update = ros::Time::now();
      ROS_INFO("[results_vis] finished attempt %d/%d at adding new subscribers", update_count, attempts);
    }

    // visualizations

    // goals

    Marker goal = create_marker("goals", 0, Marker::POINTS);
    goal.scale.x = 0.6;
    goal.scale.y = 0.6;
    goal.color.r = 1.0;  // yellow
    goal.color.g = 1.0;
    goal.color.a = 1.0;
    for (const auto& pair : goal_map)
      goal.points.push_back(set_z(pair.second, 0.01));

    if (goal.points.size() > 0)
      vis_pub.publish(goal);

    // positions / labels / lines

    Marker lines = create_marker("lines", 0, Marker::LINE_LIST);
    lines.scale.x = 0.2;
    lines.color.r = 0.2;
    lines.color.g = 0.2;
    lines.color.b = 0.2;
    lines.color.a = 1.0;

    for (const auto& pair : pose_map) {
      geometry_msgs::Point pt = set_z(pair.second, 0.25);

      if (goal_map.count(pair.first) != 0) {
        lines.points.push_back(pt);
        lines.points.push_back(set_z(goal_map[pair.first], 0.0));
      }

      Marker text = create_marker("text", pair.first, Marker::TEXT_VIEW_FACING);
      text.scale.z = 2.0;
      text.color.a = 1.0;
      text.text = std::to_string(pair.first);
      text.pose.position = set_z(pt, 0.75);

      vis_pub.publish(text);
    }

    if (lines.points.size() > 0)
      vis_pub.publish(lines);

    loop_rate.sleep();
  }

  return 0;
}
