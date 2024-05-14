#include <or_planner/or_planner.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "or_planner_node");

  ros::NodeHandle nh, pnh("~");

  or_planner::ORPlanner planner(nh, pnh);
  planner.run();

  return EXIT_SUCCESS;
}
