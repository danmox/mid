#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <or_protocol_msgs/RoutingTable.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>


typedef Eigen::Array2d Point2d;


struct RGBA
{
  double r, g, b, a;
};


std_msgs::ColorRGBA ros_color(const RGBA& c)
{
  std_msgs::ColorRGBA ros_color;
  ros_color.r = c.r;
  ros_color.g = c.g;
  ros_color.b = c.b;
  ros_color.a = c.a;
  return ros_color;
}


geometry_msgs::Point ros_point(const Point2d& eig_pt, const double z = 0.0)
{
  geometry_msgs::Point ros_pt;
  ros_pt.x = eig_pt.x();
  ros_pt.y = eig_pt.y();
  ros_pt.z = z;
  return ros_pt;
}


// matplotlib tab10
std::vector<RGBA>
  colors = {
    RGBA{0.12156862745098039,  0.4666666666666667,   0.7058823529411765, 1.0},
    RGBA{1.0,                  0.4980392156862745, 0.054901960784313725, 1.0},
    RGBA{0.17254901960784313,  0.6274509803921569,  0.17254901960784313, 1.0},
    RGBA{0.8392156862745098,  0.15294117647058825,   0.1568627450980392, 1.0},
    RGBA{0.5803921568627451,    0.403921568627451,   0.7411764705882353, 1.0},
    RGBA{0.5490196078431373,  0.33725490196078434,  0.29411764705882354, 1.0},
    RGBA{0.8901960784313725,   0.4666666666666667,   0.7607843137254902, 1.0},
    RGBA{0.4980392156862745,   0.4980392156862745,   0.4980392156862745, 1.0},
    RGBA{0.7372549019607844,   0.7411764705882353,  0.13333333333333333, 1.0},
    RGBA{0.09019607843137255,  0.7450980392156863,   0.8117647058823529, 1.0}};


ros::Publisher vis_pub;
or_protocol_msgs::NetworkStatusConstPtr status_ptr;
std::unordered_set<int> mid_ids;
std::unordered_map<int, Point2d> node_points;

double stride = 0.1;  // meters between lines in rviz
double offset = 0.5;  // distance between robot and line


void state_cb(const or_protocol_msgs::NetworkStatusConstPtr& msg)
{
  status_ptr = msg;
  for (const auto& pt : msg->positions)
    node_points[pt.node] = Point2d{pt.x, pt.y};
}


void table_cb(const or_protocol_msgs::RoutingTableConstPtr& msg)
{
  if (!status_ptr) {
    ROS_WARN("[network_viz] waiting for status message");
    return;
  }

  int flow_count = 0;
  std::unordered_map<int, std::unordered_map<int, std::vector<int>>> flow_map;
  for (const or_protocol_msgs::RoutingSrcEntry& src_entry : msg->entries) {
    const int src_id = src_entry.src_id;

    // we only care about flows between task agents
    if (mid_ids.count(src_id) != 0)
      continue;

    for (const or_protocol_msgs::RoutingDestEntry& dst_entry : src_entry.entries) {
      const int dst_id = dst_entry.dest_id;

      // we only care about flows between task agents
      if (mid_ids.count(dst_id) != 0)
        continue;

      for (const or_protocol_msgs::RoutingRule& rule : dst_entry.rules) {
        for (const int relay : rule.relays) {
          if (relay != 0) {
            flow_map[(int)rule.relay_id][relay].push_back(flow_count % colors.size());
          } else {
            break;
          }
        }
      }

      flow_count++;
    }
  }

  if (flow_count > (int)colors.size()) {
    ROS_WARN("[network_viz] more flows than colors!");
  }

  visualization_msgs::Marker flow_viz;
  flow_viz.header.frame_id = "world";
  flow_viz.action = visualization_msgs::Marker::ADD;
  flow_viz.type = visualization_msgs::Marker::LINE_LIST;
  flow_viz.pose.orientation.w = 1.0;
  flow_viz.scale.x = 0.1;

  for (const auto& src_pair : flow_map) {
    int src = src_pair.first;
    for (const auto& dst_pair : src_pair.second) {
      int dst = dst_pair.first;
      const std::vector<int>& line_colors = dst_pair.second;

      double span = line_colors.size() * stride;
      Point2d diff = (node_points[dst] - node_points[src]);
      Point2d dir = diff.matrix().normalized();
      Point2d perp = Point2d{-dir(1), dir(0)};
      Point2d anchor = node_points[src] + offset * dir - (span + stride / 2) * perp;

      double length = diff.matrix().norm() - 2.0 * offset;
      for (int i = 0; i < (int)line_colors.size(); i++) {
        Point2d line0 = anchor + i * stride * perp;
        Point2d line1 = line0 + length * dir;
        flow_viz.points.push_back(ros_point(line0));
        flow_viz.points.push_back(ros_point(line1));
        flow_viz.colors.push_back(ros_color(colors[line_colors[i]]));
        flow_viz.colors.push_back(ros_color(colors[line_colors[i]]));
      }
    }
  }

  vis_pub.publish(flow_viz);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "routing_visualizer");

  ros::NodeHandle nh, pnh("~");

  XmlRpc::XmlRpcValue mid_agent_ids_param;
  if (!nh.getParam("/mid_agent_ids", mid_agent_ids_param)) {
    ROS_FATAL("[network_viz] failed to fetch required param '/mid_agent_ids'");
    return EXIT_FAILURE;
  } else {
    for (int i = 0; i < mid_agent_ids_param.size(); i++)
      mid_ids.insert(mid_agent_ids_param[i]);
  }

  if (!pnh.getParam("offset", offset))
    ROS_WARN("[network_viz] using default value of %.2f for 'offset'", offset);
  if (!pnh.getParam("stride", stride))
    ROS_WARN("[network_viz] using default value of %.2f for 'stride'", stride);

  ros::Subscriber state_sub = nh.subscribe("state", 10, state_cb);
  ros::Subscriber table_sub = nh.subscribe("table", 10, table_cb);
  vis_pub = nh.advertise<visualization_msgs::Marker>("network_viz", 10);

  ros::spin();

  return 0;
}
