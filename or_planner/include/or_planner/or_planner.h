#ifndef OR_PLANNER_OR_PLANNER_H_
#define OR_PLANNER_OR_PLANNER_H_

#include <unordered_set>

#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <fmt/format.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <experiment_msgs/Command.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <or_protocol_msgs/NetworkStatus.h>
#include <or_protocol_msgs/RoutingTable.h>
#include <scarab_msgs/MoveAction.h>
#include <sensor_msgs/LaserScan.h>


namespace or_planner {


#define OP_DEBUG(fmt, ...) ROS_DEBUG("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_INFO(fmt, ...) ROS_INFO("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_WARN(fmt, ...) ROS_WARN("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_ERROR(fmt, ...) ROS_ERROR("[ORPlanner] " fmt, ##__VA_ARGS__)
#define OP_FATAL(fmt, ...) ROS_FATAL("[ORPlanner] " fmt, ##__VA_ARGS__)


typedef Eigen::Vector2d Point;
typedef Eigen::MatrixXd Matrix;

using std::vector;


class LinearChannel
{
 public:
  LinearChannel(const double max_range_) :
    max_range(max_range_){};

  // compute the derivative of the channel probability function w.r.t x1
  // TODO what if channel state information does not agree?
  Point derivative(const Point& x1, const Point& x2);

  // compute the probability of delivery between two nodes
  double predict(const Point& x1, const Point& x2);

  // compute the ETX between two nodes
  double etx(const Point& x1, const Point& x2);

  Matrix predict(const Matrix& poses);

 private:
  double max_range;
};


template <typename T>
std::string set_to_str(const T& set)
{
  std::string set_str;
  for (const auto& item : set)
    set_str += fmt::format("{}, ", item);
  return set_str.substr(0, set_str.size() - 2);
}


class ORPlanner
{
 public:
  ORPlanner() {}  // for testing only
  ORPlanner(ros::NodeHandle& _nh, ros::NodeHandle& _pnh);

  void table_cb(const or_protocol_msgs::RoutingTableConstPtr& msg);
  void status_cb(const or_protocol_msgs::NetworkStatusConstPtr& msg);
  void command_cb(const experiment_msgs::CommandConstPtr& msg);

  void run();

  friend class ORPlannerTest;

 private:
  ros::Subscriber map_sub, pose_sub, scan_sub, table_sub, status_sub, command_sub;
  ros::Publisher viz_pub, goal_pub;
  std::unique_ptr<ros::NodeHandle> nh, pnh;

  experiment_msgs::Command command;

  bool run_node, table_ready, status_ready, offline;

  int node_id, num_agents, root_idx, gradient_steps;

  double goal_threshold, gradient_step_size;
  std::string nav_frame;

  std::unordered_set<int> task_ids, mid_ids;
  std::unordered_set<int> task_idxs, mid_idxs;
  std::unordered_map<unsigned int, int> id_to_idx;

  Matrix poses;
  vector<vector<vector<int>>> flows;

  std::unique_ptr<LinearChannel> channel_model;

  bool found_goal;
  Point goal_pos;

  typedef actionlib::SimpleActionClient<scarab_msgs::MoveAction> ScarabMoveClient;
  std::unique_ptr<ScarabMoveClient> hfn;

  // planner goals are computed in the world frame but hfn goals must be sent in
  // the map frame
  tf2_ros::Buffer tf_buff;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  double delivery_prob(const Matrix& poses);

  Point compute_gradient(const Matrix& config);
  vector<vector<double>> compute_flow_probs(const Matrix& links);

  void send_hfn_goal(const Point& new_goal_pos);
};


class ORPlannerTest
{
  public:
  void initializeORPlanner(ORPlanner& planner, const int node_id, const double max_range,
                           const vector<int>& task_ids, const vector<int>& mid_ids)
  {
    planner.node_id = node_id;
    planner.channel_model.reset(new LinearChannel(max_range));
    planner.task_ids.insert(task_ids.begin(), task_ids.end());
    planner.mid_ids.insert(mid_ids.begin(), mid_ids.end());
  }

  Point compute_gradient(ORPlanner& planner) { return planner.compute_gradient(planner.poses); }
};


}  // namespace or_planner


#endif
