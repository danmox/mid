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


geometry_msgs::Point ros_point(const Point2d& eig_pt, const double z)
{
  geometry_msgs::Point ros_pt;
  ros_pt.x = eig_pt.x();
  ros_pt.y = eig_pt.y();
  ros_pt.z = z;
  return ros_pt;
}


// matplotlib viridis
std::vector<RGBA>
prob_colors = {
  RGBA{0.267004, 0.004874, 0.329415, 1.0}, RGBA{0.277018, 0.050344, 0.375715, 1.0},
  RGBA{0.282327, 0.094955, 0.417331, 1.0}, RGBA{0.282884, 0.13592 , 0.453427, 1.0},
  RGBA{0.278826, 0.17549 , 0.483397, 1.0}, RGBA{0.270595, 0.214069, 0.507052, 1.0},
  RGBA{0.258965, 0.251537, 0.524736, 1.0}, RGBA{0.244972, 0.287675, 0.53726 , 1.0},
  RGBA{0.229739, 0.322361, 0.545706, 1.0}, RGBA{0.214298, 0.355619, 0.551184, 1.0},
  RGBA{0.19943 , 0.387607, 0.554642, 1.0}, RGBA{0.185556, 0.41857 , 0.556753, 1.0},
  RGBA{0.172719, 0.448791, 0.557885, 1.0}, RGBA{0.160665, 0.47854 , 0.558115, 1.0},
  RGBA{0.149039, 0.508051, 0.55725 , 1.0}, RGBA{0.13777 , 0.537492, 0.554906, 1.0},
  RGBA{0.127568, 0.566949, 0.550556, 1.0}, RGBA{0.120565, 0.596422, 0.543611, 1.0},
  RGBA{0.120638, 0.625828, 0.533488, 1.0}, RGBA{0.132268, 0.655014, 0.519661, 1.0},
  RGBA{0.157851, 0.683765, 0.501686, 1.0}, RGBA{0.196571, 0.711827, 0.479221, 1.0},
  RGBA{0.24607 , 0.73891 , 0.452024, 1.0}, RGBA{0.304148, 0.764704, 0.419943, 1.0},
  RGBA{0.369214, 0.788888, 0.382914, 1.0}, RGBA{0.440137, 0.811138, 0.340967, 1.0},
  RGBA{0.515992, 0.831158, 0.294279, 1.0}, RGBA{0.595839, 0.848717, 0.243329, 1.0},
  RGBA{0.678489, 0.863742, 0.189503, 1.0}, RGBA{0.762373, 0.876424, 0.137064, 1.0},
  RGBA{0.845561, 0.887322, 0.099702, 1.0}, RGBA{0.926106, 0.89733 , 0.104071, 1.0}};

// matplotlib tab10
std::vector<RGBA>
route_colors = {
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


or_protocol_msgs::NetworkStatusConstPtr status_ptr;
std::unordered_set<int> mid_ids;
std::unordered_map<int, Point2d> node_points;
std::unordered_map<int, std::unordered_map<int, double>> probs;
std::unordered_map<int, std::unordered_map<int, std::vector<int>>> flow_map;


void state_cb(const or_protocol_msgs::NetworkStatusConstPtr& msg)
{
  status_ptr = msg;
  for (const auto& pt : msg->positions)
    node_points[pt.node] = Point2d{pt.x, pt.y};

  probs.clear();
  for (const auto& tx_node : msg->etx_table) {
    for (const auto& rx_node : tx_node.etx_list) {
      probs[(int)tx_node.node][(int)rx_node.node] = 1 / rx_node.etx;
    }
  }
}


void table_cb(const or_protocol_msgs::RoutingTableConstPtr& msg)
{
  static ros::Time last_warn(0);

  if (!status_ptr) {
    if ((ros::Time::now() - last_warn).toSec() > 5.0) {
      ROS_WARN("[network_viz] waiting for status message");
      last_warn = ros::Time::now();
    }
    return;
  }

  int flow_count = 0;
  flow_map.clear();
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
            flow_map[(int)rule.relay_id][relay].push_back(flow_count % route_colors.size());
          } else {
            break;
          }
        }
      }

      flow_count++;
    }
  }

  if (flow_count > (int)route_colors.size()) {
    ROS_WARN("[network_viz] more flows than colors!");
  }
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

  double stride = 0.1;     // meters between lines in rviz
  double offset = 0.5;     // distance between robot and line
  double z_val = 0.3;      // z coordinate to give to each line endpoint
  bool show_flows = true;  // plot packet routes in addition to link prob

  if (!pnh.getParam("offset", offset))
    ROS_WARN("[network_viz] using default value of %.2f for 'offset'", offset);
  if (!pnh.getParam("stride", stride))
    ROS_WARN("[network_viz] using default value of %.2f for 'stride'", stride);
  if (!pnh.getParam("z_value", z_val))
    ROS_WARN("[network_viz] using default value of %.2f for 'z_value'", z_val);
  if (!pnh.getParam("show_flows", show_flows))
    ROS_WARN("[network_viz] using default value of %s for 'show_flows'", show_flows ? "TRUE" : "FALSE");

  ros::Subscriber state_sub = nh.subscribe("state", 10, state_cb);
  ros::Subscriber table_sub = nh.subscribe("table", 10, table_cb);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("network_viz", 10);

  ros::Rate loop_rate(5);
  while (ros::ok()) {

    loop_rate.sleep();
    ros::spinOnce();

    if (!status_ptr) {
      ROS_WARN_THROTTLE(5.0, "[network_viz] waiting for status message");
      continue;
    }

    visualization_msgs::Marker flow_viz;
    flow_viz.header.frame_id = "world";
    flow_viz.action = visualization_msgs::Marker::ADD;
    flow_viz.type = visualization_msgs::Marker::LINE_LIST;
    flow_viz.pose.orientation.w = 1.0;
    flow_viz.scale.x = 0.1;

    for (const auto& src_pair : node_points) {
      int src = src_pair.first;
      for (const auto& dst_pair : node_points) {
        int dst = dst_pair.first;

        if (src == dst)
          continue;

        Point2d diff = (node_points[dst] - node_points[src]);
        Point2d dir = diff.matrix().normalized();
        Point2d center = node_points[src] + offset * dir;
        double length = diff.matrix().norm() - 2.0 * offset;

        if (probs[src][dst] < 1e-5 && probs[dst][src] < 1e-5)
          continue;

        // link prob
        flow_viz.points.push_back(ros_point(center, z_val));
        flow_viz.points.push_back(ros_point(center + length * dir, z_val));
        flow_viz.colors.push_back(ros_color(RGBA{1, 0, 0, probs[src][dst]}));
        flow_viz.colors.push_back(ros_color(RGBA{1, 0, 0, probs[dst][src]}));
        //int src_prob_idx = probs[src][dst] * (prob_colors.size() - 1);
        //int dst_prob_idx = probs[dst][src] * (prob_colors.size() - 1);
        //flow_viz.colors.push_back(ros_color(prob_colors[src_prob_idx]));
        //flow_viz.colors.push_back(ros_color(prob_colors[dst_prob_idx]));

        // routes
        if (show_flows && flow_map.count(src) != 0 && flow_map[src].count(dst) != 0) {
          const std::vector<int>& line_colors = flow_map[src][dst];

          double span = line_colors.size() * stride;
          Point2d perp = Point2d{-dir(1), dir(0)};
          Point2d anchor = center - (span + stride) * perp;

          for (int i = 0; i < (int)line_colors.size(); i++) {
            Point2d line0 = anchor + i * stride * perp;
            Point2d line1 = line0 + length * dir;
            flow_viz.points.push_back(ros_point(line0, z_val));
            flow_viz.points.push_back(ros_point(line1, z_val));
            flow_viz.colors.push_back(ros_color(route_colors[line_colors[i]]));
            flow_viz.colors.push_back(ros_color(route_colors[line_colors[i]]));
          }
        }
      }
    }

    if (flow_viz.points.size() > 0)
      vis_pub.publish(flow_viz);
  }

  return 0;
}
