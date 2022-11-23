#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>

#include <or_protocol/or_node.h>
#include <or_protocol_msgs/Packet.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>


volatile bool run = true;
int recv_msgs = 0;


void ping_recv(or_protocol_msgs::Packet& msg, int node_id, int bytes)
{
  ros::Time recv_stamp = ros::Time::now();

  if (msg.header.dest_id != node_id ||
      msg.header.msg_type != or_protocol_msgs::Header::PING_RES)
    return;

  std_msgs::Time send_stamp_msg;
  or_protocol::deserialize(send_stamp_msg, msg.data.data(), msg.data.size());

  double ms = (recv_stamp - send_stamp_msg.data).toSec() * 1000;
  printf("%d bytes from 192.168.0.%d: seq=%d, hops=%d, time=%.2f ms\n",
         bytes, msg.header.src_id, msg.header.seq, msg.header.hops, ms);

  recv_msgs++;
}


void handler(int s)
{
  ROS_DEBUG("[main] received shutdown signal %d", s);
  run = false;
}


int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cout << "[main] usage: ping <ip address> <node id>" << std::endl;
    return 0;
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  ros::Time::init();

  or_protocol::ORNode or_node(argv[1], 4568);
  or_node.register_recv_func(ping_recv);

  // build message
  or_protocol_msgs::Packet msg;
  msg.header.msg_type = or_protocol_msgs::Header::PING_REQ;
  msg.header.dest_id = std::stoi(argv[2]);

  int sent_msgs = 0;
  ros::Time start = ros::Time::now();
  while (run && or_node.is_running()) {
    msg.data.clear();
    msg.header.hops = 0;
    std_msgs::Time now;
    now.data = ros::Time::now();
    ros::SerializedMessage sermsg = ros::serialization::serializeMessage(now);
    msg.data.insert(msg.data.end(), sermsg.buf.get(), sermsg.buf.get() + sermsg.num_bytes);
    or_node.send(msg);
    sent_msgs++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  int total_time = (ros::Time::now() - start).toSec() * 1000;
  printf("\n%d packets transmitted, %d packets received, %d%% packet loss, %d ms\n",
         sent_msgs, recv_msgs, (sent_msgs - recv_msgs) * 100 / sent_msgs,
         total_time);
  return 0;
}
