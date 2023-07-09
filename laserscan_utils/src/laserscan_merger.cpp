#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>


typedef sensor_msgs::LaserScan Scan;


ros::Publisher scan_pub;


bool approx_equal(double a, double b)
{
  return abs(a - b) < 1e-10;
}


void scan_cb(const Scan::ConstPtr& scan1, const Scan::ConstPtr& scan2)
{
  // NOTE only operate on scans that are roughly the same (i.e. their starting
  // angle and incrememnt are the same but the total number of elements in each
  // need not be)
  if (!approx_equal(scan1->angle_min, scan2->angle_min) ||
      !approx_equal(scan1->angle_increment, scan2->angle_increment)) {
    ROS_FATAL("[laserscan_merger] scans do not have the same geometry!");
    ROS_FATAL("[laserscan_merger] scan1: {angle_min: %f, angle_increment: %f}", scan1->angle_min, scan1->angle_increment);
    ROS_FATAL("[laserscan_merger] scan2: {angle_min: %f, angle_increment: %f}", scan2->angle_min, scan2->angle_increment);
    exit(EXIT_FAILURE);
  }

  // the scans must be in the same coordinate frame in order to be comparable
  if (scan1->header.frame_id != scan2->header.frame_id) {
    ROS_FATAL("[laserscan_merger] scans are not in the same frame! (scan1 frame: %s, scan2 frame: %s)", scan1->header.frame_id.c_str(), scan2->header.frame_id.c_str());
    exit(EXIT_FAILURE);
  }

  Scan::ConstPtr small, large;
  if (scan1->ranges.size() < scan2->ranges.size()) {
    small = scan1;
    large = scan2;
  } else {
    small = scan2;
    large = scan1;
  }

  Scan::Ptr merged(new Scan(*large));
  for (size_t i = 0; i < small->ranges.size(); i++) {
    if (std::isfinite(merged->ranges[i]) && isfinite(small->ranges[i])) {
      merged->ranges[i] = std::min(merged->ranges[i], small->ranges[i]);
    } else if (std::isfinite(small->ranges[i])) {
      merged->ranges[i] = small->ranges[i];
    }
  }

  scan_pub.publish(merged);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_merger");

  ros::NodeHandle nh;

  scan_pub = nh.advertise<Scan>("merged_scan", 4);

  message_filters::Subscriber<Scan> sub1(nh, "scan1", 1);
  message_filters::Subscriber<Scan> sub2(nh, "scan2", 1);

  typedef message_filters::sync_policies::ApproximateTime<Scan, Scan> Policy;
  message_filters::Synchronizer<Policy> sync(Policy(10), sub1, sub2);

  namespace ph = std::placeholders;
  sync.registerCallback(std::bind(&scan_cb, ph::_1, ph::_2));

  ros::spin();

  return 0;
}
