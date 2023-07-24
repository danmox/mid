#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <experiment_msgs/Command.h>
#include <scarab_msgs/MoveAction.h>


bool new_command = false;
experiment_msgs::Command cmd;


void cmd_cb(const experiment_msgs::CommandConstPtr& msg)
{
  if (cmd.action != msg->action)
    new_command = true;
  cmd = *msg;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_sender");

  ros::NodeHandle nh, pnh("~");

  std::string nav_frame;
  if (!pnh.getParam("nav_frame", nav_frame)) {
    ROS_FATAL("[goal_sender] failed to get required param 'nav_frame'");
    return EXIT_FAILURE;
  }

  std::string goal_param;
  if (!pnh.getParam("goal_param", goal_param)) {
    ROS_FATAL("[goal_sender] failed to get required param 'goal_param'");
    return EXIT_FAILURE;
  }

  double goal_x, goal_y;
  if (!nh.getParam(goal_param + std::string("/x"), goal_x) ||
      !nh.getParam(goal_param + std::string("/y"), goal_y)) {
    ROS_FATAL("[goal_sender] failed to get required params %s/{x,y}", goal_param.c_str());
    return EXIT_FAILURE;
  }

  ros::Subscriber cmd_sub = nh.subscribe("/command", 1, cmd_cb);

  actionlib::SimpleActionClient<scarab_msgs::MoveAction> hfn("move", true);
  ROS_INFO("[goal_sender] waiting for hfn action server to start");
  hfn.waitForServer();
  ROS_INFO("[goal_sender] hfn action server ready");

  tf2_ros::Buffer tf_buff;
  tf2_ros::TransformListener tf_listener(tf_buff);

  ros::Rate tf_rate(10);
  while (!tf_buff.canTransform("world", nav_frame, ros::Time(0))) {
    ROS_INFO_THROTTLE(2.0, "[goal_sender] waiting for transforms to be available");
    tf_rate.sleep();
  }
  ROS_INFO("[goal_sender] transforms ready");

  // prepare goal msg

  geometry_msgs::PoseStamped world_goal_pose;
  world_goal_pose.header.frame_id = "world";
  world_goal_pose.header.stamp = ros::Time::now();
  world_goal_pose.pose.position.x = goal_x;
  world_goal_pose.pose.position.y = goal_y;
  world_goal_pose.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped local_goal_pose;
  try {
    local_goal_pose = tf_buff.transform(world_goal_pose, nav_frame);
  } catch (tf2::TransformException& ex) {
    ROS_FATAL("[goal_sender] failed to transform nav goal: %s", ex.what());
    return EXIT_FAILURE;
  }

  scarab_msgs::MoveGoal goal;
  goal.target_poses.push_back(local_goal_pose);

  // wait for signal to send goal

  ros::Rate loop_rate(5);
  while (ros::ok()) {
    ros::spinOnce();

    if (new_command && cmd.action == experiment_msgs::Command::START) {
      ROS_INFO("[goal_sender] sending goal to HFN");
      hfn.sendGoal(goal);
      new_command = false;
    } else if (new_command && cmd.action == experiment_msgs::Command::RETURN) {
      geometry_msgs::PoseStamped return_goal;
      return_goal.header.frame_id = nav_frame;
      return_goal.header.stamp = ros::Time::now();
      return_goal.pose.orientation.w = 1.0;

      scarab_msgs::MoveGoal return_hfn_goal;
      return_hfn_goal.target_poses.push_back(return_goal);

      ROS_INFO("[goal_sender] sending return goal to HFN");
      hfn.sendGoal(return_hfn_goal);
      new_command = false;
    } else if (new_command && cmd.action == experiment_msgs::Command::STOP) {
      if (hfn.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        ROS_INFO("[goal_sender] stopping robot");
        hfn.cancelGoal();
      }
      new_command = false;
    }

    loop_rate.sleep();
  }

  return 0;
}
