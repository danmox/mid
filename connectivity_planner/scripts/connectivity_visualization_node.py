#!/usr/bin/env python3

import numpy as np
import re
import rospy

from functools import partial
from threading import Lock

from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from connectivity_planner.channel_model import PiecewisePathLossModel
from connectivity_planner.connectivity_optimization import ConnectivityOpt


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

        # need to enforce thread safety since pose information will be updated
        # in the subscriber callback but accessed in the main loop
        self.pose_lock = Lock()

        transmit_power = rospy.get_param("~transmit_power", 0.0)
        self.channel_model = PiecewisePathLossModel(print_values=False, t=transmit_power)

        self.robot_poses = {}

        # robot ID map
        robot_ids = self.params['comm_ids'] + self.params['task_ids']
        self.robot_id_to_idx = {robot_id: i for i, robot_id in enumerate(robot_ids)}

        # optional params
        self.pose_topic = rospy.get_param('~pose_topic', '/unity_command/ground_truth/{}/pose')
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.ms = rospy.get_param('~marker_scale', 10)
        self.ls = rospy.get_param('~line_scale', 1)
        self.lifetime = rospy.get_param('~lifetime', 1)

        self.pose_subs = []
        for id in robot_ids:
            self.pose_subs.append(
                rospy.Subscriber(
                    self.pose_topic.format(id),
                    PoseStamped,
                    partial(self.pose_cb, id),
                    queue_size=1
                )
            )

        # self.tf_sub = rospy.Subscriber('tf_relay', TFMessage, self.tf_cb, queue_size=10)
        self.viz_pub = rospy.Publisher('connectivity_marker', Marker, queue_size=1)
        self.conn_pub = rospy.Publisher('connectivity', Marker, queue_size=1)

    def process_pose(self, pose, robot_id, timestamp):

        # publish pose marker

        marker_msg = Marker()

        marker_msg.header.frame_id = self.frame_id
        marker_msg.header.stamp = timestamp
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
            return

        self.viz_pub.publish(marker_msg)

        # update internal robot pose used for drawing connectivity lines

        trans = pose.position
        with self.pose_lock:
            self.robot_poses[robot_id] = {'pose': np.asarray((trans.x, trans.y, trans.z)),
                                          'stamp': timestamp}

    def pose_cb(self, robot_id, pose_msg):
        self.process_pose(pose_msg.pose, robot_id, pose_msg.header.stamp)

    def tf_cb(self, tf_msg):

        for tf in tf_msg.transforms:

            robot_id = re.match('^quad[0-9]+', tf.child_frame_id)
            if robot_id is None or robot_id[0] not in self.robot_id_to_idx:
                continue
            robot_id = robot_id[0]

            trans = tf.transform.translation
            pose = Pose(Point(trans.x, trans.y, trans.z), Quaternion(0,0,0,1))

            self.process_pose(pose, robot_id, tf.header.stamp)

    def run_node(self):

        msg = Marker()
        msg.header.frame_id = self.frame_id
        msg.type = Marker.LINE_LIST
        msg.action = Marker.MODIFY
        msg.lifetime = rospy.Duration(self.lifetime)
        msg.scale.x = self.ls
        msg.color = ColorRGBA(0,0,0,0.9)
        msg.ns = 'conn'
        msg.id = 0

        loop_rate = rospy.Rate(30)
        while self.run and not rospy.is_shutdown():

            loop_rate.sleep()

            # we only want to publish links between agents with active pose information
            # TODO publish on a link by link basis?
            now = rospy.get_rostime()
            active_poses = []
            with self.pose_lock:
                for key in self.robot_poses.keys():
                    if (now - self.robot_poses[key]['stamp']).to_sec() < self.lifetime:
                        active_poses.append(self.robot_poses[key]['pose'])
                    else:
                        rospy.logdebug(f"skipped pose with time diff: {(now - self.robot_poses[key]['stamp']).to_sec()} seconds")

            if len(active_poses) < 2:
                rospy.loginfo('no active poses. skipping this iteration.')
                continue

            msg.points = []

            np_poses = np.vstack(active_poses)
            rate, _ = self.channel_model.predict(np_poses[:,:2])

            for i in range(rate.shape[0]):
                for j in range(i+1, rate.shape[0]):
                    if rate[i,j] > 0.0:
                        msg.points.append(Point(*np_poses[i,:]))
                        msg.points.append(Point(*np_poses[j,:]))

            self.conn_pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('connectivity_visualization_node')
        viz = ConnectivityViz()
        viz.run_node()
    except rospy.ROSInterruptException:
        pass
