#include <curses.h>
#include <experiment_msgs/Command.h>
#include <ros/ros.h>
#include <string.h>


using namespace std;
using experiment_msgs::Command;


std::string to_string(const Command& command)
{
  switch(command.action) {
    case Command::START:
      return std::string("START");
    case Command::STOP:
      return std::string("STOP");
    case Command::RETURN:
      return std::string("RETURN");
    default:
      return std::string("INVALID");
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "commander");
  ros::NodeHandle nh;
  ros::Publisher command_pub = nh.advertise<Command>("/command", 1);

  initscr();
  clear();
  noecho();
  cbreak();
  timeout(50); // wait 50 ms
  nodelay(stdscr, TRUE);

  int row = 0;
  mvprintw(row++, 0, "publishing to %s", command_pub.getTopic().c_str());
  mvprintw(row++, 0, "use 's' to start, SPC to stop, 'r' to return, and 'q' to quit");

  Command command_msg;
  command_msg.action = Command::STOP;

  double command_delay{5.0};
  ros::Time last_command{0};

  int ch;
  ros::Rate loop_rate(10);

  while (nh.ok()) {
    row = 2;
    mvprintw(row++, 0, "sending command: %s   ", to_string(command_msg).c_str());
    mvprintw(row++, 0, "");
    refresh();

    ch = getch();

    bool new_command = false;
    if (ch != ERR) {
      if (ch == 's') {
        new_command = command_msg.action != Command::START;
        command_msg.action = Command::START;
      } else if (ch == 'r') {
        new_command = command_msg.action != Command::RETURN;
        command_msg.action = Command::RETURN;
      } else if (ch == ' ') {
        new_command = command_msg.action != Command::STOP;
        command_msg.action = Command::STOP;
      } else if (ch == 'q') {
        command_msg.action = Command::STOP;
        command_pub.publish(command_msg);
        ros::Duration(0.5).sleep();
        break;
      }
    }

    if (new_command || (ros::Time::now() - last_command).toSec() > command_delay) {
      command_pub.publish(command_msg);
      last_command = ros::Time::now();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  endwin();

  return 0;
}
