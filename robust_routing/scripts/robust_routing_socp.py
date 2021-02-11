#!/usr/bin/env python

import rospy
import robust_routing.socp as socp
from robust_routing.utils import parse_rate_graph
from routing_msgs.msg import *


class RobustRoutingNode():
    def __init__(self):
        super().__init__()

        self.comm_spec = None
        self.reqs_sub = rospy.Subscriber('comm_reqs', CommSpec, self.set_comm_spec)
        self.rate_sub = rospy.Subscriber('rate_graph', RateGraph, self.solve, queue_size=1)
        self.route_pub = rospy.Publisher('robust_routes', RobustRoutes)

    def set_comm_spec(self, msg):
        self.comm_spec = msg

    def solve(self, rate_graph):

        if self.comm_spec is None:
            rospy.logwarn('no communication requirements received: no routes will be published')
            return

        # TODO check if names in comm_reqs are in rate_graph

        name2idx, idx2name, rate_mean, rate_var = parse_rate_graph(rate_graph)

        slack, routes, status = socp.solve(comm_spec.flows, rate_mean, rate_var, name2idx)
        if status != 'optimal':
            rospy.logerror('robust routing problem returned with status: {}'.format(status))
            return

        # publish routing update

        routing_msg = RobustRoutes()
        routing_msg.stamp = rospy.get_rostime()
        routing_msg.flows = []

        N = rate_mean.shape[0]
        for k in range(routes.shape[2]):
            robust_flow = RobustFlow()
            robust_flow.tx = comm_reqs[k].tx
            robust_flow.rx = comm_reqs[k].rx

            for i in range(N):
                prob_relay = ProbRelay()
                prob_relay.tx = idx2name[i]
                prob_relay.gateways = [ProbGateway(idx2name[j], routes[i,j,k]) for j in range(N) if routes[i,j,k] > 0]

            routing_msg.flows.append(robust_flow)

        self.route_pub.publish(routing_msg)


if __name__ == '__main__':

    rospy.init_node('robust_routing')
    node = RobustRoutingNode()
    rospy.spin()
