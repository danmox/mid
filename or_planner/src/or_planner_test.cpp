#include <filesystem>

#include <fmt/format.h>
#include <or_planner/or_planner.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseStamped.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <or_protocol_msgs/RoutingTable.h>


namespace fs = std::filesystem;


int main(int argc, char** argv)
{
  if (argc != 2) {
    fmt::print("usage: or_planner_test <bagfile>\n");
    return EXIT_FAILURE;
  }

  // import bag data
  fs::path bagfile(argv[1]);
  if (!fs::exists(bagfile)) {
    fmt::print("provided bagfile {} doesn't exit\n", bagfile.c_str());
    return EXIT_FAILURE;
  }

  rosbag::Bag bag;
  bag.open(bagfile.c_str(), rosbag::bagmode::Read);

  geometry_msgs::PoseStampedConstPtr pose_msg;
  or_protocol_msgs::NetworkStatusConstPtr network_msg;
  or_protocol_msgs::RoutingTableConstPtr routing_msg;
  for (const rosbag::MessageInstance& m : rosbag::View(bag)) {
    if (m.getDataType() == "geometry_msgs/PoseStamped")
      pose_msg =  m.instantiate<geometry_msgs::PoseStamped>();
    else if (m.getDataType() == "or_protocol_msgs/NetworkStatus")
      network_msg = m.instantiate<or_protocol_msgs::NetworkStatus>();
    else if (m.getDataType() == "or_protocol_msgs/RoutingTable")
      routing_msg = m.instantiate<or_protocol_msgs::RoutingTable>();
    else
      fmt::print("unknown datatype {}\n", m.getDataType());
  }

  or_planner::ORPlanner planner;
  or_planner::ORPlannerTest tester;
  tester.initializeORPlanner(planner, 42, 30.0, {40, 41}, {42});

  planner.pose_cb(pose_msg);
  planner.status_cb(network_msg);
  planner.table_cb(routing_msg);

  or_planner::Vec2d grad = tester.compute_gradient(planner);

  fmt::print("grad: ({}, {})\n", grad(0), grad(1));

  return 0;
}
