#!/usr/bin/env python3

import rospy
import numpy as np

import robust_routing.socp as socp
from robust_routing.utils import parse_rate_graph, socp_info
from routing_msgs.msg import *


class RobustRoutingNode():
    def __init__(self):
        super().__init__()

        self.comm_spec = None

        self.reqs_sub = rospy.Subscriber('robust_routing/comm_spec', CommSpec, self.set_comm_spec)
        self.rate_sub = rospy.Subscriber('rate_graph', RateGraph, self.solve, queue_size=1)
        self.route_pub = rospy.Publisher('robust_routing/routes', RobustRoutes)

        self.print_info = rospy.get_param('~socp_info', False)

        # the value at which to consider a route to be zero
        self.route_threshold = rospy.get_param('~route_threshold', 1e-5)
        rospy.loginfo(f'using a route threshold of {self.route_threshold}')

    def set_comm_spec(self, spec):
        rospy.loginfo(f'received CommSpec with {len(spec.flows)} flows')
        self.comm_spec = spec

    def solve(self, rate_graph):

        if self.comm_spec is None:
            rospy.logwarn('no communication requirements received: no routes will be published')
            return

        #
        # prepare inputs and solve robust routing problem
        #

        # NOTE robust routing optimization will fail if any variance value is 0; a hack used here is
        # to to set the variance to 1 bps which is extremely low for a channel with at least 1 Mbps
        # of bandwidth, functionally making the variance zero
        variance_warning = False
        for edge in rate_graph.edges:
            if not edge.rate_variance > 0.0:
                if not variance_warning:
                    variance_warning = True
                    rospy.logwarn('edge.rate_variance of 0 detected: setting to 1bps')
                edge.rate_variance = 1  # bps (extremely low variance)

        # convert rate_graph message to rate and variance matrices used in the optimization
        name2idx, idx2name, rate_mean, rate_var = parse_rate_graph(rate_graph)

        # all names in cached comm_spec must be present in rate_graph
        spec_names = set([f.rx for f in self.comm_spec.flows] + [f.tx for f in self.comm_spec.flows])
        for name in spec_names:
            if name not in name2idx:
                rospy.logerr(f'{name} in CommSpec but not in RateGraph: no routes will be published')
                return

        slack, routes, status = socp.solve(self.comm_spec.flows, rate_mean, rate_var, name2idx)
        if status != 'optimal':
            rospy.logerr(f'robust routing problem returned with status: {status}')
            return

        #
        # convert optimization output into routing update message
        #

        routing_msg = RobustRoutes()
        routing_msg.stamp = rospy.get_rostime()
        routing_msg.flows = []

        N = rate_mean.shape[0]
        for k, flow in enumerate(self.comm_spec.flows):
            robust_flow = RobustFlow()
            robust_flow.tx = flow.tx
            robust_flow.rx = flow.rx
            robust_flow.relays = []

            for i in set(range(N)) - {name2idx[flow.rx]}:  # skip Rx node

                # to be a valid relay, node i must meet at least one of the following criterion:
                #   1) be a source node
                #   2) have some incoming traffic
                #
                # NOTE: the robust routing problem is formulated in such a way that non-source nodes
                # can have outgoing traffic without any incoming traffic ("phantom flows"); while
                # this does no harm, these nodes can safely be ignored in the routing update message
                # to reduce unnecessary bloat
                if i != name2idx[flow.tx] and np.max(routes[:,i,k]) < self.route_threshold:
                    continue

                valid_gateways = routes[i,:,k] > self.route_threshold
                if np.sum(valid_gateways) == 0:
                    rospy.logerr(
                        f'no valid gateways at {idx2name[i]} despite appreciable incoming traffic '
                        f'(Tx max: {np.max(routes[i,:,k])}, Rx max: {np.max(routes[:,i,k])}, flow: '
                        f'{flow.tx} -> {flow.rx} @ {flow.rate}) be sure route_threshold is set '
                        f'appropriately (route_threshold: {self.route_threshold})')
                    return

                prob_relay = ProbRelay()
                prob_relay.tx = idx2name[i]
                prob_relay.gateways = []

                tx_total = np.sum(routes[i,valid_gateways,k])
                for j in valid_gateways.nonzero()[0]:
                    prob_relay.gateways += [ProbGateway(idx2name[j], routes[i,j,k] / tx_total)]

                robust_flow.relays.append(prob_relay)

            routing_msg.flows.append(robust_flow)

        self.route_pub.publish(routing_msg)

        # print routes

        if self.print_info:
            socp_info(routes, self.comm_spec.flows, idx2name, rate_mean, rate_var)


if __name__ == '__main__':

    rospy.init_node('robust_routing')
    node = RobustRoutingNode()
    rospy.spin()
