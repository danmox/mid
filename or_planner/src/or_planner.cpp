#include <stack>
#include <unordered_set>

#include <or_planner/or_planner.h>
#include <or_protocol/constants.h>
#include <or_protocol/network_state.h>
#include <tf2/utils.h>

#include <exploration_msgs/Goal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <or_protocol_msgs/Header.h>
#include <visualization_msgs/Marker.h>


namespace or_planner {


Vec2d LinearChannel::derivative(const Vec2d& x1, const Vec2d& x2)
{
  Vec2d diff = x1 - x2;
  double dist = diff.norm();
  if (dist > max_range)
    return Vec2d::Zero();
  return - diff / dist / max_range;
}


double LinearChannel::predict(const Vec2d& x1, const Vec2d& x2)
{
  return std::max(1.0 - (x1 - x2).norm() / max_range, 0.0);
}


double LinearChannel::etx(const Vec2d& x1, const Vec2d& x2)
{
  double prob = std::max(1.0 - (x1 - x2).norm() / max_range, 0.0);
  return prob < 1.0 / or_protocol::ETX_MAX ? or_protocol::ETX_MAX : 1.0 / prob;
}


Eigen::MatrixXd LinearChannel::predict(const Eigen::MatrixXd& poses)
{
  Eigen::MatrixXd link_probs = Eigen::MatrixXd::Zero(poses.cols(), poses.cols());
  for (int i = 0; i < poses.cols(); i++) {
    for (int j = 0; j < poses.cols(); j++) {
      if (i == j)
        continue;
      link_probs(i,j) = predict(poses.col(i), poses.col(j));
    }
  }
  return link_probs;
}


ORPlanner::ORPlanner(ros::NodeHandle& _nh, ros::NodeHandle& _pnh) :
  run_node(false), table_ready(false), status_ready(false), found_goal(false)
{
  // wait for a signal to start running
  command.action = experiment_msgs::Command::STOP;

  nh.reset(new ros::NodeHandle(_nh));
  pnh.reset(new ros::NodeHandle(_pnh));
  hfn.reset(new ScarabMoveClient("move", true));

  tf_listener.reset(new tf2_ros::TransformListener(tf_buff));

  gradient_steps = 20;
  if (!pnh->getParam("gradient_steps", gradient_steps)) {
    OP_WARN("failed to fetch param 'gradient_steps', using default value %d", gradient_steps);
  }

  gradient_step_size = 20.0;
  if (!pnh->getParam("gradient_step_size", gradient_step_size)) {
    OP_WARN("failed to fetch param 'gradient_step_size', using default value %.2f", gradient_step_size);
  }

  status_sub = nh->subscribe("status", 1, &ORPlanner::status_cb, this);
  command_sub = nh->subscribe("/command", 1, &ORPlanner::command_cb, this);

  viz_pub = nh->advertise<visualization_msgs::Marker>("planner", 1);
  goal_pub = nh->advertise<exploration_msgs::Goal>("goals", 1);

  if (!pnh->getParam("node_id", node_id)) {
    OP_FATAL("failed to fetch required param 'node_id'");
    return;
  }

  std::string robot_type;
  if (!pnh->getParam("robot_type", robot_type)) {
    OP_FATAL("failed to fetch required param 'node_id'");
    return;
  }

  double max_range;
  if (!nh->getParam("/max_range", max_range)) {
    OP_FATAL("failed to fetch required param '/max_range'");
    return;
  } else {
    channel_model.reset(new LinearChannel(max_range));
  }

  XmlRpc::XmlRpcValue task_agent_ids_param;
  if (!nh->getParam("/task_agent_ids", task_agent_ids_param)) {
    OP_FATAL("failed to fetch required param '/task_agent_ids'");
    return;
  } else {
    for (int i = 0; i < task_agent_ids_param.size(); i++)
      task_ids.insert(task_agent_ids_param[i]);
  }

  XmlRpc::XmlRpcValue mid_agent_ids_param;
  if (!nh->getParam("/mid_agent_ids", mid_agent_ids_param)) {
    OP_FATAL("failed to fetch required param '/mid_agent_ids'");
    return;
  } else {
    for (int i = 0; i < mid_agent_ids_param.size(); i++)
      mid_ids.insert(mid_agent_ids_param[i]);
  }

  // if the robot type is not set correctly by the user bad things happen
  if (robot_type == "mid") {
    if (mid_ids.count(node_id) == 0) {
      OP_FATAL("robot_type is 'mid' but 'node_id' (%d) is not in mid_ids: {%s}; are /mid_agent_ids and /task_agent_ids set correctly?", node_id, set_to_str(mid_ids).c_str());
      return;
    }
  } else if (robot_type != "none" && robot_type != "task") {
    OP_FATAL("robot_type '%s' not set correctly for agent %d", robot_type.c_str(), node_id);
    return;
  }

  goal_threshold = 1.5;
  if (!pnh->getParam("goal_threshold", goal_threshold))
    OP_WARN("failed to fetch param 'goal_threshold': using default value of %.2f m", goal_threshold);

  if (!pnh->getParam("nav_frame", nav_frame)) {
    OP_FATAL("failed to fetch required param 'nav_frame'");
    return;
  }

  OP_INFO("waiting for hfn action server to start");
  hfn->waitForServer();
  OP_INFO("hfn action server ready");

  ros::Rate tf_rate(10);
  while (!tf_buff.canTransform("world", nav_frame, ros::Time(0))) {
    OP_INFO("waiting for transforms to be available");
    tf_rate.sleep();
  }
  OP_INFO("transforms ready");

  run_node = true;
}


void ORPlanner::table_cb(const or_protocol_msgs::RoutingTableConstPtr& msg)
{
  if (!status_ready)
    return;

  typedef std::vector<int> int_vec;
  typedef or_protocol_msgs::Header::_relays_type RelayArray;

  auto id_arr_to_idx_vec = [&](const RelayArray& arr) {
    int_vec relay_vec;
    size_t i = 0;
    while (i < arr.size() && arr[i] != 0)
      relay_vec.push_back(id_to_idx.at(arr[i++]));
    return relay_vec;
  };

  // compute all possible packet paths through the network with the given
  // routing table

  flows.clear();
  for (const or_protocol_msgs::RoutingSrcEntry& src_entry : msg->entries) {
    const int src_idx = id_to_idx.at(src_entry.src_id);

    // we only care about flows between task agents
    if (mid_idxs.count(src_idx) != 0)
      continue;

    for (const or_protocol_msgs::RoutingDestEntry& dst_entry : src_entry.entries) {
      const int dst_idx = id_to_idx.at(dst_entry.dest_id);

      // we only care about flows between task agents
      if (mid_idxs.count(dst_idx) != 0)
        continue;

      std::unordered_map<int, int_vec> relay_map;
      for (const or_protocol_msgs::RoutingRule& rule : dst_entry.rules)
        relay_map[id_to_idx.at(rule.relay_id)] = id_arr_to_idx_vec(rule.relays);

      // compute all paths for the given flow

      std::vector<int_vec> paths{int_vec{src_idx}};
      std::stack<int> incomplete({static_cast<int>(paths.size() - 1)});
      while (!incomplete.empty()) {
        int idx = incomplete.top();
        incomplete.pop();

        int last_node = paths[idx].back();
        if (last_node == dst_idx)
          continue;

        if (relay_map.count(last_node) == 0 || relay_map[last_node].size() == 0) {
          OP_FATAL("last_node (%d) != dst_idx (%d) but has no neighbors", last_node, dst_idx);
          OP_FATAL("path: {%s}", set_to_str(paths[idx]).c_str());
          std::string map_str = "";
          for (const auto& item : id_to_idx)
            map_str += fmt::format("{}: {}, ", item.first, item.second);
          OP_FATAL("id_to_idx: {%s}", map_str.substr(0, map_str.size()-2).c_str());
          ROS_FATAL_STREAM("[ORPlanner] dst_entry:" << std::endl << dst_entry);
          ROS_FATAL_STREAM("[ORPlanner] src_entry:" << std::endl << src_entry);
          table_ready = run_node = false;
          return;
        }
        const int_vec& relays = relay_map[last_node];

        const int_vec path_so_far = paths[idx];

        paths[idx].push_back(relays[0]);
        incomplete.push(idx);

        for (size_t i = 1; i < relays.size(); i++) {
          paths.push_back(path_so_far);
          paths.back().push_back(relays[i]);
          incomplete.push(paths.size()-1);
        }
      }

      flows.push_back(paths);
    }
  }

  table_ready = true;
}


void ORPlanner::status_cb(const or_protocol_msgs::NetworkStatusConstPtr& msg)
{
  // NOTE msg->positions does not always contain position information for all
  // the nodes represented in msg->etx_table and the routing table, primarily at
  // startup; perform additional checks to ensure valid data before proceeding
  // with calculations

  std::unordered_set<int> etx_id_set;
  for (const or_protocol_msgs::ETXList& table : msg->etx_table) {
    etx_id_set.insert(table.node);
    for (const or_protocol_msgs::ETXEntry& entry : table.etx_list) {
      etx_id_set.insert(entry.node);
    }
  }

  std::unordered_set<int> pos_id_set;
  for (const or_protocol_msgs::Point& pt : msg->positions)
    pos_id_set.insert(pt.node);

  bool valid_set = true;
  if (etx_id_set.size() != pos_id_set.size()) {
    valid_set = false;
  } else {
    for (int id : etx_id_set) {
      if (pos_id_set.count(id) == 0) {
        valid_set = false;
        break;
      }
    }
  }
  if (!valid_set) {
    OP_WARN("NetworkStatus message does not contain valid set of positions and etx values");
    status_ready = false;
    return;
  }

  // data checks have passed, construct id to idx translations and etx table

  int idx = 0;
  id_to_idx.clear();
  for (const or_protocol_msgs::Point &pt : msg->positions)
    id_to_idx[pt.node] = idx++;

  for (const int id : task_ids)
    task_idxs.insert(id_to_idx[id]);
  for (const int id : mid_ids)
    mid_idxs.insert(id_to_idx[id]);

  num_agents = id_to_idx.size();
  root_idx = id_to_idx.at(node_id);

  poses = Eigen::MatrixXd(2, num_agents);
  for (const or_protocol_msgs::Point& pt : msg->positions) {
    const int idx = id_to_idx[pt.node];
    poses(0, idx) = pt.x;
    poses(1, idx) = pt.y;
  }

  // compute link probabilities from ETX table

  Eigen::MatrixXd link_probs = channel_model->predict(poses);

  std::unordered_set<int> node_ids = task_ids;
  node_ids.insert(mid_ids.begin(), mid_ids.end());

  // compute ETX map for NetworkState (must be in id space)
  or_protocol::ETXMap etx_map;
  for (const int src : node_ids) {
    std::unordered_map<int, double> inner_map;
    for (const int dst : node_ids) {
      if (src == dst)
        continue;
      double etx = channel_model->etx(poses.col(id_to_idx[src]), poses.col(id_to_idx[dst]));
      inner_map.emplace(dst, etx);
    }
    etx_map.emplace(src, inner_map);
  }

  // compute routing table
  or_protocol::NetworkState ns;
  ns.set_etx_map(etx_map);
  ns.update_routes(node_id);

  // set status_ready
  status_ready = true;

  // compute flow probabilities
  table_cb(ns.get_routing_table_msg());

  status_ready = true;
}


void ORPlanner::command_cb(const experiment_msgs::CommandConstPtr& msg)
{
  command = *msg;
}


// TODO numerical issues? switch to log prob?
double ORPlanner::delivery_prob(const Eigen::MatrixXd& poses)
{
  Eigen::MatrixXd links = channel_model->predict(poses);

  double prob{1.0};
  for (const std::vector<std::vector<int>>& flow : flows) {
    double flow_prob{1.0};
    for (const std::vector<int>& path : flow) {
      double path_prob{1.0};
      for (size_t i = 1; i < path.size(); i++) {
        path_prob *= links(path[i-1], path[i]);
      }
      flow_prob *= 1.0 - path_prob;
    }
    prob *= 1.0 - flow_prob;
  }

  return prob;
}


std::vector<std::vector<double>>
ORPlanner::compute_flow_probs(const Eigen::MatrixXd& links)
{
  std::vector<std::vector<double>> flow_probs;
  for (const std::vector<std::vector<int>>& flow : flows) {
    std::vector<double> path_probs;
    for (const std::vector<int>& path : flow) {
      double path_prob{1.0};
      for (size_t i = 1; i < path.size(); i++) {
        path_prob *= links(path[i - 1], path[i]);
      }
      path_probs.push_back(path_prob);
    }
    flow_probs.push_back(path_probs);
  }

  return flow_probs;
}


Vec2d ORPlanner::compute_gradient(const Eigen::MatrixXd& team_config)
{
  Eigen::MatrixXd link_probs = channel_model->predict(team_config);
  std::vector<std::vector<double>> flow_probs = compute_flow_probs(link_probs);

  Vec2d gradient = Vec2d::Zero();

  for (size_t j = 0; j < flows.size(); j++) {
    const std::vector<std::vector<int>>& paths = flows[j];
    const std::vector<double>& path_probs = flow_probs[j];

    for (size_t i = 0; i < paths.size(); i++) {
      const std::vector<int>& path = paths[i];

      size_t j = 1;  // MID agents are never sources
      while (path[j] != root_idx && j < path.size() - 2)
        j++;

      // some paths in the flow don't involve root_idx
      if (path[j] != root_idx)
        continue;

      int prev_idx = path[j - 1];
      int next_idx = path[j + 1];
      double C1 = path_probs[i] / link_probs(prev_idx, root_idx);
      double C2 = path_probs[i] / link_probs(root_idx, next_idx);
      gradient += C1 * channel_model->derivative(team_config.col(root_idx), team_config.col(prev_idx)) +
                  C2 * channel_model->derivative(team_config.col(root_idx), team_config.col(next_idx));
    }
  }

  return gradient;
}


Vec2d ORPlanner::compute_goal()
{
  Vec2d goal;

  int max_steps = 20;
  for (int i = 0; i < max_steps; ++i) {
  }

  return goal;
}


void ORPlanner::send_hfn_goal(const Eigen::Array2d& new_goal_pos)
{
  if (found_goal && (new_goal_pos - goal_pos).matrix().norm() < goal_threshold) {
    OP_DEBUG("new_goal (%.2f, %.2f) near previous goal (%.2f, %.2f): not sending HFN command", new_goal_pos(0), new_goal_pos(1), goal_pos(0), goal_pos(1));
  } else {

    goal_pos = new_goal_pos;
    found_goal = true;

    // send goal to HFN (must be transformed from world frame to navigation
    // - i.e. map - frame)

    geometry_msgs::PoseStamped world_goal_pose;
    world_goal_pose.header.frame_id = "world";
    world_goal_pose.header.stamp = ros::Time::now();
    world_goal_pose.pose.position.x = goal_pos(0);
    world_goal_pose.pose.position.y = goal_pos(1);
    world_goal_pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped local_goal_pose;
    try {
      local_goal_pose = tf_buff.transform(world_goal_pose, nav_frame);
    } catch (tf2::TransformException& ex) {
      OP_ERROR("failed to transform nav goal: %s", ex.what());
      return;
    }

    scarab_msgs::MoveGoal goal;
    goal.target_poses.push_back(local_goal_pose);

    OP_INFO("sending goal (%.2f, %.2f) to HFN", local_goal_pose.pose.position.x, local_goal_pose.pose.position.y);

    hfn->sendGoal(goal);

    // publish world frame goal for eventual visualization

    exploration_msgs::Goal vis_goal;
    vis_goal.goal.x = goal_pos(0);
    vis_goal.goal.y = goal_pos(1);
    vis_goal.stamp = ros::Time::now();
    goal_pub.publish(vis_goal);
  }
}


void ORPlanner::run()
{
  ros::Rate rate(5);
  while (run_node && ros::ok()) {
    rate.sleep();
    ros::spinOnce();

    bool skip_loop = true;
    if (command.action == experiment_msgs::Command::STOP) {
      ROS_WARN_THROTTLE(5.0, "[ORPlanner] command action is: STOP");
      if (hfn->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        OP_INFO("cancelling current goal");
        hfn->cancelGoal();
      }
    } else if (command.action == experiment_msgs::Command::RETURN) {
      ROS_WARN_THROTTLE(5.0, "[ORPlanner] command action is: RETURN");
      send_hfn_goal(Eigen::Array2d{0.0, 0.0});
    } else {
      skip_loop = false;
    }

    // clear visualizations if we're not running the planner this iteration
    if (skip_loop) {
      visualization_msgs::Marker grad_msg;
      grad_msg.header.frame_id = "world";
      grad_msg.ns = "grad";
      grad_msg.id = 0;
      grad_msg.action = visualization_msgs::Marker::DELETE;

      visualization_msgs::Marker plan_msg;
      plan_msg.header.frame_id = "world";
      plan_msg.ns = "plan";
      plan_msg.id = 0;
      plan_msg.action = visualization_msgs::Marker::DELETE;

      viz_pub.publish(grad_msg);
      viz_pub.publish(plan_msg);

      continue;
    }

    if (!status_ready) {
      ROS_WARN_THROTTLE(1.0, "[ORPlanner] waiting for NetworkStatus");
      continue;
    }
    if (!table_ready) {
      ROS_WARN_THROTTLE(1.0, "[ORPlanner] waiting for RoutingTable");
      continue;
    }

    // TODO move to centroid of task agents?
    if (flows.size() == 0) {
      ROS_WARN_THROTTLE(1.0, "robot not involved in any flows: navigating to centroid of task team");
      Vec2d centroid = Vec2d::Zero();
      for (int i : task_idxs)
        centroid += poses.col(i);
      centroid /= task_idxs.size();
      send_hfn_goal(centroid);
      continue;
    }

    // compute gradient across flows

    Eigen::MatrixXd next_config = poses;
    for (int step = 0; step < gradient_steps; step++) {
      Vec2d gradient = compute_gradient(next_config);
      next_config.col(root_idx) += gradient_step_size * gradient;
    }

    // compute control actions

    double current_prob = delivery_prob(next_config);
    if (current_prob < 1e-5)
      OP_WARN("low delivery probability %f", current_prob);

    send_hfn_goal(next_config.col(root_idx).array());

    // planner visualizations

    geometry_msgs::Point p0;
    p0.x = poses(0, root_idx);
    p0.y = poses(1, root_idx);
    p0.z = 0.1;

    geometry_msgs::Point p1;
    p1.x = goal_pos(0);
    p1.y = goal_pos(1);
    p1.z = 0.1;

    visualization_msgs::Marker plan_msg;
    plan_msg.header.frame_id = "world";
    plan_msg.ns = "plan";
    plan_msg.id = 0;
    plan_msg.type = visualization_msgs::Marker::LINE_STRIP;
    plan_msg.action = visualization_msgs::Marker::ADD;
    plan_msg.lifetime = ros::Duration(1.0);
    plan_msg.color.b = 1;
    plan_msg.color.a = 1;
    plan_msg.points.push_back(p0);
    plan_msg.points.push_back(p1);
    plan_msg.scale.x = 0.1;
    plan_msg.pose.orientation.w = 1.0;  // avoid rviz warning

    visualization_msgs::Marker goal_msg;
    goal_msg.header.frame_id = "world";
    goal_msg.ns = "goal";
    goal_msg.id = 0;
    goal_msg.type = visualization_msgs::Marker::POINTS;
    goal_msg.action = visualization_msgs::Marker::ADD;
    goal_msg.lifetime = ros::Duration(1.0);
    goal_msg.color.b = 1;
    goal_msg.color.a = 1;
    goal_msg.points.push_back(p1);
    goal_msg.scale.x = 0.4;
    goal_msg.scale.y = 0.4;
    goal_msg.pose.orientation.w = 1.0;  // avoid rviz warning

    viz_pub.publish(plan_msg);
    viz_pub.publish(goal_msg);
  }
}


}  // namespace or_planner
