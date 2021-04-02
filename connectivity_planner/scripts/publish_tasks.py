#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Point

rospy.init_node("publish_task")

positions = rospy.get_param("~positions")
pubs = [rospy.Publisher(f"/connectivity_planner/task{i+1}/pose", PoseStamped, queue_size=1, latch=True) for i in range(len(positions))]

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    for i, pos in enumerate(positions):
        pose_stamped = PoseStamped()
        position = Point(*pos)
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position = position
        pubs[i].publish(pose_stamped)
    rate.sleep()