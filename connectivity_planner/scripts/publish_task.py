#!/usr/bin/env python3
import argparse
import rospy
from geometry_msgs.msg import PoseStamped, Point

parser = argparse.ArgumentParser(description='Publish to connectivity_planner task agent pose.')
parser.add_argument('task_id', type=int, help='The task agent id. Starting with zero.')
parser.add_argument('x', type=float)
parser.add_argument('y', type=float)
parser.add_argument('z', type=float)

args = parser.parse_args()

rospy.init_node("publish_task")
pub = rospy.Publisher(f"/connectivity_planner/task{args.task_id}/pose", PoseStamped, queue_size=1)

pose_stamped = PoseStamped()
position = Point(args.x, args.y, args.z)
pose_stamped.pose.position = position

pub.publish(pose_stamped)