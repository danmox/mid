#!/usr/bin/env python

import rospy
import numpy as np
from connectivity_planner.channel_model import PiecewisePathLossModel
from connectivity_planner.connectivity_optimization import ConnectivityOpt
from geometry_msgs.msg import PoseStamped, Pose
from abc import ABC, abstractmethod
from functools import partial
from typing import List, Union

class ConnectivityPlanner(ABC):
    def __init__(self) -> None:
        self.altitude = rospy.get_param("~/altitude", 30)

        n_comm = rospy.get_param("~/n_comm")
        n_task = rospy.get_param("~/n_task")
        if n_comm is None or n_task is None:
            raise ValueError("Must specify `num_comm_agents` and `num_task_agents`. Cannot be None.")

        comm_pose_fmt = rospy.get_param("~/comm_pose_fmt", "~/comm{}/pose")
        comm_cmd_pose_fmt = rospy.get_param("~/comm_cmd_pose_fmt", "~/comm{}/cmd_pose")
        task_pose_fmt = rospy.get_param("~/task_pose_fmt", "~/task{}/pose")

        self.N0 = rospy.get_param("~/N0", -70.0)
        self.n = rospy.get_param("~/n", 10)
        self.L0 = rospy.get_param("~/L0", -53.0)
        self.a = rospy.get_param("~/a", 0.2)
        self.b = rospy.get_param("~/b", 6.0)

        self.x_comm = np.zeros((self.n_comm, 2))
        self.x_task = np.zeros((self.n_comm, 2))

        self.channel_model = PiecewisePathLossModel(print_values=False, n0=-self.N0, n=self.n, l0=self.L0, a=self.a, b=self.b)

        self.task_pose_subs: List[rospy.Subscriber] = []
        self.comm_pose_subs: List[rospy.Subscriber] = []
        self.comm_cmd_pose_pubs: List[rospy.Publisher] = []
        for i in range(self.n_task):
            self.task_pose_subs.append(rospy.Subscriber(comm_pose_fmt.format(i), PoseStamped, partial(self.pose_callback, i, "task"), queue_size=1))
        for i in range(self.n_comm):
            self.comm_pose_subs.append(rospy.Subscriber(comm_cmd_pose_fmt.format(i), PoseStamped, partial(self.pose_callback, i, "comm"), queue_size=1))
            self.comm_cmd_pose_pubs.append(rospy.Subscriber(task_pose_fmt.format(i)))

    def pose_callback(self, i, type, pose_stamped: PoseStamped):
        x = pose_to_numpy(pose_stamped)[:2]
        if type is "task":
            self.x_task[i,:] = x
        elif type is "comm":
            self.x_comm[i,:] = x
        else:
            rospy.logerr(f"Unexpected pose callback type {type}.")

    @abstractmethod
    def update():
        pass

    def run(self):
        while not rospy.is_shutdown():
            self.update()

class OptimizationPlanner(ConnectivityPlanner):
    def __init__(self) -> None:
        super().__init__()
        self.max_step_size = rospy.get_param("~/max_step_size", 0.5)
        self.min_step_size = rospy.get_param("~/min_step_size", 0.01)
        self.step_size = self.max_step_size

    def update(self):
        conn_opt = ConnectivityOpt(self.channel_model, self.x_task, self.x_comm)
        connectivity, success = conn_opt.update_network_config(self.step_size)

        # Failure most likely due to too aggresive step size.
        # Increase step size on success. Decrease on failure.
        if success:
            self.step_size = min(self.max_step_size, 1.2 * self.step_size)
        else:
            self.step_size = max(self.min_step_size, 0.75*self.step_size)
        rospy.logdebug(f"Optimizer success: {success}")
        rospy.logdebug(f"Connectivity, step size: {connectivity:.2f} | {self.step_size:.2f}")

        self.x_comm = conn_opt.get_comm_config()
        for i, pub in enumerate(self.comm_cmd_pose_pubs):
            position = [self.x_comm[i,0], self.x_comm[i,1], self.altitude]
            pose_stamped = PoseStamped(pose=Pose(position=position))
            pub.publish(pose_stamped)

class CNNPlanner(ConnectivityPlanner):
    def __init__(self) -> None:
        pass

def pose_to_numpy(pose: Union[Pose, PoseStamped]) -> np.ndarray:
    if isinstance(pose, PoseStamped):
        pose = pose.pose
    point = pose.position
    return np.asarray([point.x, point.y, point.z])

if __name__ == "__main__":
    try:
        rospy.init_node("connectivity_planner")

        type = rospy.get_param("~/type", "optimization")
        if type is "optimization":
            planner = OptimizationPlanner()
        elif type is "cnn":
            planner = CNNPlanner()
        else:
            raise ValueError(f"Unknown planner of type {type}.")
        planner.run()
    except rospy.ROSInterruptException:
        pass