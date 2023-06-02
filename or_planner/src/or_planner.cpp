#include <stack>
#include <unordered_set>

#include <or_planner/or_planner.h>
#include <tf2/utils.h>

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
  nh(_nh), pnh(_pnh), run_node(false), table_ready(false), status_ready(false)
{
  table_sub = nh.subscribe("table", 1, &ORPlanner::table_cb, this);
  status_sub = nh.subscribe("status", 1, &ORPlanner::status_cb, this);
  pose_sub = nh.subscribe("pose", 1, &ORPlanner::pose_cb, this);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  viz_pub = nh.advertise<visualization_msgs::Marker>("planner", 1);

  if (!pnh.getParam("node_id", node_id)) {
    OP_FATAL("failed to fetch required param 'node_id'");
    return;
  }

  double max_range;
  if (!nh.getParam("/max_range", max_range)) {
    OP_FATAL("failed to fetch required param '/max_range'");
    return;
  } else {
    channel_model.reset(new LinearChannel(max_range));
  }

  XmlRpc::XmlRpcValue task_agent_ids_param;
  if (!nh.getParam("/task_agent_ids", task_agent_ids_param)) {
    OP_FATAL("failed to fetch required param '/task_agent_ids'");
    return;
  } else {
    for (int i = 0; i < task_agent_ids_param.size(); i++)
      task_ids.insert(task_agent_ids_param[i]);
  }

  XmlRpc::XmlRpcValue mid_agent_ids_param;
  if (!nh.getParam("/mid_agent_ids", mid_agent_ids_param)) {
    OP_FATAL("failed to fetch required param '/mid_agent_ids'");
    return;
  } else {
    for (int i = 0; i < mid_agent_ids_param.size(); i++)
      mid_ids.insert(mid_agent_ids_param[i]);
  }

  if (!pnh.getParam("v_max", v_max)) {
    v_max = 0.5;
    OP_WARN("failed to fetch param 'v_max': using default value of %.2f m/s", v_max);
  }

  if (!pnh.getParam("w_max", w_max)) {
    w_max = 1.5;
    OP_WARN("failed to fetch param 'w_max': using default value of %.2f rad/s",
            w_max);
  }

  if (!pnh.getParam("heading_tol", heading_tol)) {
    heading_tol = 0.25;  // ~15 deg
    OP_WARN("failed to fetch param 'heading_tol': using default value of %.2f rad", heading_tol);
  }

  if (!pnh.getParam("position_tol", position_tol)) {
    position_tol = 0.25;
    OP_WARN("failed to fetch param 'position_tol': using default value of %.2f m", position_tol);
  }

  run_node = true;
}


void ORPlanner::table_cb(const or_protocol_msgs::RoutingTableConstPtr& msg)
{
  typedef std::vector<int> int_vec;
  typedef or_protocol_msgs::Header::_relays_type RelayArray;

  auto id_arr_to_idx_vec = [&](const RelayArray& arr) {
    int_vec relay_vec;
    size_t i = 0;
    while (i < arr.size() && arr[i] != 0)
      relay_vec.push_back(id_to_idx.at(arr[i++]));
    return relay_vec;
  };

  // compute all routes and their probabilities

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
      std::vector<double> path_probs{1.0};
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
        const int prob_so_far = path_probs[idx];

        paths[idx].push_back(relays[0]);
        path_probs[idx] *= link_probs(last_node, relays[0]);

        for (size_t i = 1; i < relays.size(); i++) {
          paths.push_back(path_so_far);
          paths.back().push_back(relays[i]);
          path_probs.push_back(prob_so_far * link_probs(last_node, relays[i]));
          incomplete.push(paths.size()-1);
        }
      }

      flows.push_back(paths);
      flow_probs.push_back(path_probs);
    }
  }

  table_ready = true;
}


void ORPlanner::status_cb(const or_protocol_msgs::NetworkStatusConstPtr& msg)
{
  // NOTE assuming positions is a complete set of the ids used in etx / routing tables
  // TODO construct incrementally?

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

  link_probs = Eigen::MatrixXd::Zero(num_agents, num_agents);
  for (const or_protocol_msgs::ETXList& table : msg->etx_table) {
    const int src_idx = id_to_idx.at(table.node);
    for (const or_protocol_msgs::ETXEntry& entry : table.etx_list) {
      const int dst_idx = id_to_idx.at(entry.node);
      link_probs(src_idx, dst_idx) = 1.0 / entry.etx;
    }
  }

  status_ready = true;
}


void ORPlanner::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!pose_ptr)
    pose_ptr.reset(new geometry_msgs::Pose2D);
  pose_ptr->x = msg->pose.position.x;
  pose_ptr->y = msg->pose.position.y;
  pose_ptr->theta = tf2::getYaw(msg->pose.orientation);
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


void ORPlanner::run()
{
  ros::Rate rate(10);
  while (run_node && ros::ok()) {
    rate.sleep();
    ros::spinOnce();

    if (!pose_ptr) {
      ROS_WARN_THROTTLE(1.0, "[ORPlanner] waiting for robot pose");
      continue;
    }

    if (!table_ready || !status_ready) {
      ROS_WARN_THROTTLE(1.0, "[ORPlanner] waiting for RoutingTable or NetworkStatus");
      continue;
    }

    // compute gradient across flows

    Vec2d gradient = Vec2d::Zero();
    for (size_t j = 0; j < flows.size(); j++) {
      const std::vector<std::vector<int>>& paths = flows[j];
      const std::vector<double>& path_probs = flow_probs[j];

      for (size_t i = 0; i < paths.size(); i++) {
        const std::vector<int> &path = paths[i];

        size_t j = 1; // MID agents are never sources
        while (path[j] != root_idx && j < path.size() - 1)
          j++;

        // some paths in the flow don't involve root_idx
        if (path[j] != root_idx)
          continue;

        int prev_idx = path[j - 1];
        int next_idx = path[j + 1];
        double C1 = path_probs[i] / link_probs(prev_idx, root_idx);
        double C2 = path_probs[i] / link_probs(root_idx, next_idx);
        gradient += C1 * channel_model->derivative(poses.col(root_idx), poses.col(prev_idx)) +
                    C2 * channel_model->derivative(poses.col(root_idx), poses.col(next_idx));
      }
    }

    // compute control actions

    if (gradient.norm() < 1e-3) {
      vel_pub.publish(geometry_msgs::Twist());
      continue;
    }
    gradient.normalize();

    double current_prob = delivery_prob(poses);
    if (current_prob < 1e-5)
      OP_WARN("low delivery probability %f", current_prob);

    // search for local optimum along gradient direction
    double step_size = 0.5;
    Eigen::MatrixXd next_config = poses;
    for (int steps = 0; steps < 10; steps++) {
      next_config.col(root_idx) += step_size * gradient;
      double next_prob = delivery_prob(next_config);
      if (next_prob < current_prob) {
        next_config.col(root_idx) -= step_size * gradient;
        step_size *= 0.5;
      }
    }
    double pos_error = (next_config.col(root_idx) - poses.col(root_idx)).norm();

    double yaw_error = atan2(gradient(1), gradient(0)) - pose_ptr->theta;
    double yaw_sign = yaw_error > 0 ? 1.0 : -1.0;
    if (yaw_error > 2.0 * M_PI) // TODO should be >=?
      yaw_error -= 2.0 * M_PI;
    if (yaw_error < -2.0 * M_PI)
      yaw_error += 2.0 * M_PI;

    geometry_msgs::Twist vel_cmd;
    if (abs(yaw_error) > heading_tol && pos_error > position_tol) {
      vel_cmd.angular.z = yaw_sign * w_max;
    } else if (pos_error > position_tol) {
      vel_cmd.angular.z = std::min(yaw_sign * w_max, 0.2 * yaw_error);
      vel_cmd.linear.x = std::min(v_max, 0.5 * pos_error);
    }
    vel_pub.publish(vel_cmd);

    // gradient visualization

    geometry_msgs::Point a0;
    a0.x = poses(0, root_idx);
    a0.y = poses(1, root_idx);
    a0.z = 0.2;

    geometry_msgs::Point a1 = a0;
    a1.x += gradient(0);
    a1.y += gradient(1);

    visualization_msgs::Marker grad_msg;
    grad_msg.header.frame_id = "world";
    grad_msg.ns = "grad";
    grad_msg.id = 0;
    grad_msg.type = visualization_msgs::Marker::ARROW;
    grad_msg.action = visualization_msgs::Marker::ADD;
    grad_msg.lifetime = ros::Duration(1.0);
    grad_msg.color.r = 1;
    grad_msg.color.a = 1;
    grad_msg.points.push_back(a0);
    grad_msg.points.push_back(a1);
    grad_msg.scale.x = 0.1;
    grad_msg.scale.y = 0.2;
    grad_msg.scale.z = 0.25;

    viz_pub.publish(grad_msg);
  }
}


}  // namespace or_planner
