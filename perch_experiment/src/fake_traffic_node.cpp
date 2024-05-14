#include <vector>

#include <ros/ros.h>

#include <experiment_msgs/Command.h>
#include <experiment_msgs/Payload.h>


experiment_msgs::Command cmd;


void cmd_cb(const experiment_msgs::CommandConstPtr& msg)
{
  cmd = *msg;
}


int main(int argc, char** argv)
{
  cmd.action = experiment_msgs::Command::STOP;

  ros::init(argc, argv, "fake_traffic_node");

  ros::NodeHandle nh, pnh("~");

  ros::Subscriber cmd_sub = nh.subscribe("/command", 1, cmd_cb);
  ros::Publisher payload_pub = nh.advertise<experiment_msgs::Payload>("traffic", 1);

  int payload_size;
  if (!pnh.getParam("payload_size", payload_size)) {
    ROS_FATAL("[fake_traffic] failed to get required param 'payload_size'");
    return EXIT_FAILURE;
  }

  double send_rate;
  if (!pnh.getParam("send_rate", send_rate)) {
    ROS_FATAL("[fake_traffic] failed to get required param 'send_rate'");
    return EXIT_FAILURE;
  }

  ROS_INFO("[fake_traffic] sending payloads: topic=%s, bytes=%d, rate=%.0fHz", payload_pub.getTopic().c_str(), payload_size, send_rate);

  experiment_msgs::Payload msg;
  msg.seq = 0;
  msg.data = std::vector<uint8_t>(payload_size, 1);

  ros::Rate loop_rate(send_rate);
  while (ros::ok()) {
    ros::spinOnce();
    if (cmd.action == experiment_msgs::Command::START) {
      payload_pub.publish(msg);
      msg.seq++;
    } else {
      ROS_WARN_THROTTLE(5.0, "[fake_traffic] waiting for start command");
    }
    loop_rate.sleep();
  }

  return 0;
}
