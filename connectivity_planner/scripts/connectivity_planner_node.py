#!/usr/bin/env python3

import json
from abc import ABC, abstractmethod
from functools import partial
from math import ceil
from os import path
from os.path import join
from typing import List, Union

import numpy as np
import rospkg
import rospy
import torch
import torch.jit
from connectivity_planner import lloyd
from connectivity_planner.channel_model import PiecewisePathLossModel
from connectivity_planner.connectivity_optimization import ConnectivityOpt
from connectivity_planner.feasibility import connect_graph
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker
from scipy.optimize import linear_sum_assignment
from scipy.spatial import distance_matrix


class ConnectivityPlanner(ABC):
    def __init__(self) -> None:
        self.altitude = rospy.get_param("~altitude", 30)
        self.rate = rospy.get_param("~rate", 1)

        if rospy.has_param("~comm_ids") and rospy.has_param("~task_ids"):
            self.comm_ids = rospy.get_param("~comm_ids")
            self.task_ids = rospy.get_param("~task_ids")
        else:
            raise ValueError(
                'failed to fetch "comm_ids" and/or "task_ids" from parameter server'
            )

        self.n_comm = len(self.comm_ids)
        self.n_task = len(self.task_ids)

        comm_pose_fmt = rospy.get_param("~comm_pose_fmt", "~comm{}/pose")
        comm_cmd_pose_fmt = rospy.get_param("~comm_cmd_pose_fmt", "~comm{}/cmd_pose")
        task_pose_fmt = rospy.get_param("~task_pose_fmt", "~task{}/pose")

        # rviz marker scale
        self.ms = rospy.get_param("~marker_scale", 1)

        self.t = rospy.get_param("~transmit_power", 0.0)
        self.N0 = rospy.get_param("~noise_floor", -70.0)
        self.n = rospy.get_param("~fading_exponent", 2.52)
        self.L0 = rospy.get_param("~L0", -53.0)
        self.a = rospy.get_param("~a", 0.2)
        self.b = rospy.get_param("~b", 6.0)

        self.channel_model = PiecewisePathLossModel(
            print_values=False, t=self.t, n0=self.N0, n=self.n, l0=self.L0, a=self.a, b=self.b
        )
        self.comm_range = self.channel_model.calculate_range(max_range=1000)
        rospy.loginfo(f"Comm range is {self.comm_range:.2f} meters.")

        self.x_comm = np.zeros((self.n_comm, 2))
        self.x_task = np.zeros((self.n_task, 2))
        self.x_comm_target = np.zeros((self.n_comm, 2))

        self.task_pose_subs: List[rospy.Subscriber] = []
        self.comm_pose_subs: List[rospy.Subscriber] = []
        self.comm_cmd_pose_pubs: List[rospy.Publisher] = []
        for i, agent_id in enumerate(self.task_ids):
            self.task_pose_subs.append(
                rospy.Subscriber(
                    task_pose_fmt.format(agent_id),
                    PoseStamped,
                    partial(self.pose_callback, i, "task"),
                    queue_size=1,
                )
            )
        for i, agent_id in enumerate(self.comm_ids):
            self.comm_pose_subs.append(
                rospy.Subscriber(
                    comm_pose_fmt.format(agent_id),
                    PoseStamped,
                    partial(self.pose_callback, i, "comm"),
                    queue_size=1,
                )
            )
            self.comm_cmd_pose_pubs.append(
                rospy.Publisher(
                    comm_cmd_pose_fmt.format(agent_id), PoseStamped, queue_size=1
                )
            )

        self.rviz_pub = rospy.Publisher("~rviz", Marker, queue_size=100)

    def pose_callback(self, i, type, pose_stamped: PoseStamped):
        x = pose_to_numpy(pose_stamped)[:2]
        if type == "task":
            self.x_task[i, :] = x
            self.rviz_pub.publish(
                ConnectivityPlanner.marker_factory(
                    "task", i, pose_stamped.pose, scale=self.ms, color=(0, 0, 1, 1)
                )
            )
        elif type == "comm":
            self.x_comm[i, :] = x
            self.rviz_pub.publish(
                ConnectivityPlanner.marker_factory(
                    "comm", i, pose_stamped.pose, scale=self.ms, color=(1, 0, 0, 1)
                )
            )
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

    def publish_single(self, i, target):
        pose = Pose(Point(target[0], target[1], self.altitude), Quaternion(0,0,0,1))
        pose_stamped = PoseStamped(pose=pose)
        pose_stamped.header.frame_id = "world"
        self.comm_cmd_pose_pubs[i].publish(pose_stamped)
        self.rviz_pub.publish(
            ConnectivityPlanner.marker_factory(
                "comm_target", i, pose_stamped.pose, scale=self.ms, color=(0, 1, 0, 1)
            )
        )

    def publish(self, x_comm_target):
        for i in range(self.n_comm):
            self.publish_single(i, x_comm_target[i, :])

    @classmethod
    def marker_factory(self, ns, i, pose, scale=1, color=(1, 0, 0, 1)):
        return Marker(
            Header(frame_id="world"),
            ns,
            i,
            Marker.SPHERE,
            Marker.MODIFY,
            pose,
            Vector3(scale, scale, scale),
            ColorRGBA(*color),
            rospy.Duration(60),
            True,
            None,
            None,
            None,
            None,
            None,
        )


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
        connectivity, success = conn_opt.update_network_config(self.step_size)
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
        super().__init__()
        self.max_steps = rospy.get_param("~max_steps", 100)
        self.model, self.params = self.load_model("convae.ts")
        self.model_scale = self.comm_range / self.params["comm_range"]
        rospy.loginfo(f"Model scale: {self.model_scale}")

        self.input_img_pub = rospy.Publisher("~input_image", Image, queue_size=1)
        self.cnn_img_pub = rospy.Publisher("~cnn_image", Image, queue_size=1)

    @staticmethod
    def load_model(model_name):
        rospack = rospkg.RosPack()
        models_dir = join(rospack.get_path("connectivity_planner"), "models")

        with open(path.join(models_dir, f"{model_name}.json")) as f:
            params = json.load(f)
        params = lloyd.compute_paramaters(params)
        model = torch.jit.load(join(models_dir, model_name))
        return model, params

    def update(self):
        # Copy to avoid race condition
        x_task = np.copy(self.x_task)
        x_comm = np.copy(self.x_comm)
        # center and rescale to model scale
        x_mean = np.mean(x_task, axis=0)
        x_task = (x_task - x_mean) / self.model_scale
        x_comm = (x_comm - x_mean) / self.model_scale

        input_img = lloyd.kernelized_config_img(x_task, self.params)
        cnn_img = (
            self.model.evaluate(torch.from_numpy(input_img)).cpu().detach().numpy()
        )
        self.input_img_pub.publish(self.numpy_to_image_msg(input_img))
        self.cnn_img_pub.publish(self.numpy_to_image_msg(cnn_img))

        # extract peaks
        config_subs, _ = lloyd.compute_peaks(cnn_img, threshold_val=60)
        x_comm_target = lloyd.sub_to_pos(
            self.params["meters_per_pixel"], self.params["img_size"][0], config_subs
        )
        # run lloyds
        lloyd_its = 0
        while lloyd_its < self.max_steps:
            voronoi_cells = lloyd.compute_voronoi(x_comm_target, self.params["bbx"])
            x_comm_target_new = lloyd.lloyd_step(
                cnn_img,
                self.params["xy"],
                x_comm_target,
                voronoi_cells,
                self.params["coverage_range"],
            )
            if np.linalg.norm(x_comm_target_new - x_comm_target) < 1e-8:
                break
            x_comm_target = x_comm_target_new
            lloyd_its += 1
        rospy.loginfo(f"converged after {lloyd_its} iterations of lloyd's algorithm")
        # assign agents to targets
        distance = distance_matrix(x_comm, x_comm_target)
        comm_idx, target_idx = linear_sum_assignment(distance)

        # transform back into original coordinates
        x_comm_target = (x_comm_target * self.model_scale) + x_mean
        for i, j in zip(comm_idx, target_idx):
            self.publish_single(i, x_comm_target[j, :])

    @staticmethod
    def numpy_to_image_msg(x):
        img = Image()
        img.header.frame_id = "world"
        img.height = x.shape[0]
        img.width = x.shape[1]
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = x.shape[1]
        img.data = list(x[:].astype(np.uint8).tobytes())
        return img


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
