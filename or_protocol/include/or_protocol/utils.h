#ifndef OR_PROTOCOL_UTILS_H_
#define OR_PROTOCOL_UTILS_H_


#include <or_protocol_msgs/Header.h>
#include <ros/console.h>


namespace or_protocol {


#define OR_DEBUG(fmt, ...) ROS_DEBUG("[ORNode] " fmt, ##__VA_ARGS__)
#define OR_INFO(fmt, ...) ROS_INFO("[ORNode] " fmt, ##__VA_ARGS__)
#define OR_ERROR(fmt, ...) ROS_ERROR("[ORNode] " fmt, ##__VA_ARGS__)


// searches through header.relays for id and returns its priority (i.e. position
// in the array); if an occurence of id is not found then the size of the array
// is returned, corresponding to the lowest priority
int relay_priority(int id, const or_protocol_msgs::Header& header);


}  // namespace or_protocol


#endif
