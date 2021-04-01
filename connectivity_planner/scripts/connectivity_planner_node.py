#!/usr/bin/env python3
import rospy
import numpy as np
from connectivity_planner.channel_model import PiecewisePathLossModel
from connectivity_planner.connectivity_optimization import ConnectivityOpt
from connectivity_planner.feasibility import connect_graph
from geometry_msgs.msg import PoseStamped, Pose, Point
from abc import ABC, abstractmethod
from functools import partial
from typing import List, Union
from math import ceil


class ConnectivityPlanner(ABC):
    def __init__(self) -> None:
        self.altitude = rospy.get_param("~altitude", 30)
        self.rate = rospy.get_param("~rate", 1)

        self.n_comm = rospy.get_param("~n_comm")
        self.n_task = rospy.get_param("~n_task")
        if self.n_comm is None or self.n_task is None:
            raise ValueError(
                "Must specify `num_comm_agents` and `num_task_agents`. Cannot be None."
            )

        comm_pose_fmt = rospy.get_param("~comm_pose_fmt", "~comm{}/pose")
        comm_cmd_pose_fmt = rospy.get_param("~comm_cmd_pose_fmt", "~comm{}/cmd_pose")
        task_pose_fmt = rospy.get_param("~task_pose_fmt", "~task{}/pose")

        self.N0 = rospy.get_param("~N0", -70.0)
        self.n = rospy.get_param("~n", 2.52)
        self.L0 = rospy.get_param("~L0", -53.0)
        self.a = rospy.get_param("~a", 0.2)
        self.b = rospy.get_param("~b", 6.0)

        self.channel_model = PiecewisePathLossModel(
            print_values=False, n0=self.N0, n=self.n, l0=self.L0, a=self.a, b=self.b
        )
        self.comm_range = self.channel_model.calculate_range(max_range=1000)
        rospy.loginfo(f"Comm range is {self.comm_range:.2f} meters.")

        self.x_comm = np.zeros((self.n_comm, 2))
        self.x_task = np.zeros((self.n_task, 2))
        self.x_comm_target = np.zeros((self.n_comm, 2))

        self.task_pose_subs: List[rospy.Subscriber] = []
        self.comm_pose_subs: List[rospy.Subscriber] = []
        self.comm_cmd_pose_pubs: List[rospy.Publisher] = []
        for i in range(self.n_task):
            self.task_pose_subs.append(
                rospy.Subscriber(
                    task_pose_fmt.format(i + 1),
                    PoseStamped,
                    partial(self.pose_callback, i, "task"),
                    queue_size=1,
                )
            )
        for i in range(self.n_comm):
            self.comm_pose_subs.append(
                rospy.Subscriber(
                    comm_pose_fmt.format(i + 1),
                    PoseStamped,
                    partial(self.pose_callback, i, "comm"),
                    queue_size=1,
                )
            )
            self.comm_cmd_pose_pubs.append(
                rospy.Publisher(comm_cmd_pose_fmt.format(i), PoseStamped, queue_size=1)
            )

    def pose_callback(self, i, type, pose_stamped: PoseStamped):
        x = pose_to_numpy(pose_stamped)[:2]
        if type == "task":
            rospy.logdebug(f"Task{i}: {x}")
            self.x_task[i, :] = x
        elif type == "comm":
            self.x_comm[i, :] = x
        else:
            rospy.logerr(f"Unexpected pose callback type {type}.")

    @abstractmethod
    def update():
        pass

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def publish(self, x_comm_target):
        for i, pub in enumerate(self.comm_cmd_pose_pubs):
            point = Point(x_comm_target[i, 0], x_comm_target[i, 1], self.altitude)
            pose_stamped = PoseStamped(pose=Pose(position=point))
            pose_stamped.header.frame_id = "world"
            pub.publish(pose_stamped)


class OptimizationPlanner(ConnectivityPlanner):
    def __init__(self) -> None:
        super().__init__()
        self.max_step_size = rospy.get_param("~max_step_size", 0.5)
        self.min_step_size = rospy.get_param("~min_step_size", 0.1)
        self.step_size = self.max_step_size
        self.feasible = False  # assume infeasible originally

    def update(self):
        # copy to avoid race condition
        x_task = np.copy(self.x_task)

        # if solution is not feasible we use connect_graph to find a feasible solution
        if not self.feasible:
            rospy.logdebug(
                "Solution is not feasible, using connect_graph to enforce feasibility."
            )
            x_comm_target = connect_graph(x_task, self.comm_range)

            # Check that we have enough comm agents.
            n_comm_required = x_comm_target.shape[0]
            if n_comm_required > self.n_comm:
                rospy.logfatal(
                    f"Need at least {n_comm_required} agents. Have only {self.n_comm}. Spawning additional agents is not implemented."
                )
            elif n_comm_required == 0:
                rospy.logwarn("Zero comm agents required.")
                # No comm agents required. Distirbute agents between task positions.
                self.x_comm_target = deal_positions(self.n_comm, x_task)
                self.feasible = True
            else:
                # May have more comm agents that required. Deal them out evenly to positions.
                self.x_comm_target = deal_positions(self.n_comm, x_comm_target)
                self.feasible = True

        conn_opt = ConnectivityOpt(self.channel_model, x_task, self.x_comm_target)
        connectivity, success = conn_opt.update_network_config(
            self.step_size, verbose=True
        )
        self.x_comm_target = conn_opt.get_comm_config()
        self.publish(self.x_comm_target)
        # if connectivity is close to zero we are close to infeasability
        self.feasible = connectivity > 1e-5

        # Failure most likely due to too aggresive step size.
        # Increase step size on success. Decrease on failure.
        if success:
            self.step_size = min(self.max_step_size, 1.2 * self.step_size)
            self.x_comm_target = conn_opt.get_comm_config()
        else:
            self.step_size = max(self.min_step_size, 0.75 * self.step_size)

        rospy.logdebug(
            f"""
        Optimizer Results
        -----------------
        Success: {success}
        Connectivity: {connectivity:.2f}
        Step size: {self.step_size:.2f}
        x_task: {x_task}
        x_comm_taget: {self.x_comm_target}
        """
        )


class CNNPlanner(ConnectivityPlanner):
    def __init__(self) -> None:
        pass


def pose_to_numpy(pose: Union[Pose, PoseStamped]) -> np.ndarray:
    if isinstance(pose, PoseStamped):
        pose = pose.pose
    point = pose.position
    return np.asarray([point.x, point.y, point.z])


def deal_positions(n_agents, positions):
    """
    Deal out the agents to positions like cards. Continue dealing agents until run out of agents.
    Some positions may receive multiple agents.
    """
    n_positions = positions.shape[0]
    idx = np.arange(n_positions)
    idx = np.tile(idx, ceil(n_agents / n_positions))
    return positions[idx[:n_agents], :]


if __name__ == "__main__":
    try:
        rospy.init_node("connectivity_planner")

        planner_type = rospy.get_param("~type", "optimization")
        if planner_type == "optimization":
            planner = OptimizationPlanner()
        elif planner_type == "cnn":
            planner = CNNPlanner()
        else:
            raise ValueError(f"Unknown planner of type {type}.")
        planner.run()
    except rospy.ROSInterruptException:
        pass