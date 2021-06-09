#!/usr/bin/env python3

import numpy as np
import re
import rospy

from functools import partial
from connectivity_planner.channel_model import PiecewisePathLossModel
from connectivity_planner.connectivity_optimization import ConnectivityOpt
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class ConnectivityViz:
    def __init__(self):

        self.run = True

        required_params = ['~comm_ids', '~task_ids']
        self.params = {}
        for param in required_params:
            if rospy.has_param(param):
                self.params[param[1:]] = rospy.get_param(param)
            else:
                rospy.logfatal(f'failed to fetch ROS param: {param}')
                self.run = False
                return

        # robot ID map
        robot_ids = self.params['comm_ids'] + self.params['task_ids']
        self.robot_id_to_idx = {robot_id: i for i, robot_id in enumerate(robot_ids)}

        # optional params
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.ms = rospy.get_param('~marker_scale', 10)
        self.lifetime = rospy.get_param('~lifetime', 1)

        self.tf_sub = rospy.Subscriber('tf_relay', TFMessage, self.tf_cb, queue_size=1)
        self.viz_pub = rospy.Publisher('connectivity_marker', Marker, queue_size=10)

    def tf_cb(self, tf_msg):

        for tf in tf_msg.transforms:

            robot_id = re.match('^quad[0-9]+', tf.child_frame_id)
            if robot_id is None or robot_id[0] not in self.robot_id_to_idx:
                continue
            robot_id = robot_id[0]

            trans = tf.transform.translation
            pose = Pose(Point(trans.x, trans.y, trans.z), Quaternion(0,0,0,1))

            # get marker

            marker_msg = Marker()

            marker_msg.header.frame_id = self.frame_id
            marker_msg.header.stamp = rospy.get_rostime()
            marker_msg.id = self.robot_id_to_idx[robot_id]
            marker_msg.type = Marker.SPHERE
            marker_msg.action = Marker.MODIFY
            marker_msg.pose = pose
            marker_msg.scale = Vector3(self.ms, self.ms, self.ms)
            marker_msg.lifetime = rospy.Duration(self.lifetime)

            if robot_id in self.params['comm_ids']:
                marker_msg.ns = 'comm'
                marker_msg.color = ColorRGBA(0, 0, 1, 1)
            elif robot_id in self.params['task_ids']:
                marker_msg.ns = 'task'
                marker_msg.color = ColorRGBA(1, 0, 0, 1)
            else:
                rospy.logerr(f'{robot_id} not in comm_ids or task_ids: skipping')
                continue

            self.viz_pub.publish(marker_msg)

    def run_node(self):
        while self.run and not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('connectivity_visualization_node')
        viz = ConnectivityViz()
        viz.run_node()
    except rospy.ROSInterruptException:
        pass
