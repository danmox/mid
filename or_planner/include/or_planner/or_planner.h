#ifndef OR_PLANNER_OR_PLANNER_H_
#define OR_PLANNER_OR_PLANNER_H_


#include <Eigen/Dense>
#include <fmt/format.h>
#include <ros/ros.h>
#include <unordered_set>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <or_protocol_msgs/RoutingTable.h>
#include <sensor_msgs/LaserScan.h>

namespace or_planner {


#define OP_DEBUG(fmt, ...) ROS_DEBUG("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_INFO(fmt, ...) ROS_INFO("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_WARN(fmt, ...) ROS_WARN("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_ERROR(fmt, ...) ROS_ERROR("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_FATAL(fmt, ...) ROS_FATAL("[ORPlanner] " fmt, ##__VA_ARGS__)


typedef Eigen::Vector2d Vec2d;


class LinearChannel
{
  public:
    LinearChannel(const double max_range_) : max_range(max_range_) {};

    // compute the derivative of the channel probability function w.r.t x1
    // TODO what if channel state information does not agree?
    Vec2d derivative(const Vec2d& x1, const Vec2d& x2);

    // compute the probability of delivery between two nodes
    double predict(const Vec2d& x1, const Vec2d& x2);

    Eigen::MatrixXd predict(const Eigen::MatrixXd& poses);

  private:
    double max_range;
};


template<typename T>
std::string set_to_str(const T& set) {
  std::string set_str;
  for (const auto& item : set)
    set_str += fmt::format("{}, ", item);
  return set_str.substr(0, set_str.size()-2);
}


class ORPlanner
{
  public:
    ORPlanner(ros::NodeHandle& _nh, ros::NodeHandle& _pnh);

    void run();

  private:
    ros::Subscriber map_sub, pose_sub, scan_sub, table_sub, status_sub;
    ros::Publisher vel_pub, viz_pub;
    ros::NodeHandle nh, pnh;

    volatile bool run_node, table_ready, status_ready;

    int node_id, num_agents, root_idx;

    double v_max, w_max;
    double heading_tol, position_tol;

    std::unordered_set<int> task_ids, mid_ids;
    std::unordered_set<int> task_idxs, mid_idxs;
    std::unordered_map<unsigned int, int> id_to_idx;

    geometry_msgs::Pose2DPtr pose_ptr;

    Eigen::MatrixXd poses;
    Eigen::MatrixXd link_probs;
    std::vector<std::vector<std::vector<int>>> flows;
    std::vector<std::vector<double>> flow_probs;

    std::unique_ptr<LinearChannel> channel_model;

    double delivery_prob(const Eigen::MatrixXd& poses);

    void scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
    void map_cb(const nav_msgs::OccupancyGridConstPtr& msg);
    void table_cb(const or_protocol_msgs::RoutingTableConstPtr& msg);
    void status_cb(const or_protocol_msgs::NetworkStatusConstPtr& msg);
};


}  // namespace or_planner


#endif