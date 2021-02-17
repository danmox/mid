#!/usr/bin/env python3

import sys
import numpy as np
import robust_routing.socp as socp
import rospy
from robust_routing.utils import parse_rate_graph, socp_info
from routing_msgs.msg import FlowSpec, CommSpec, RateGraph, RateEdge


def simple_routing_test():

    # config: warty1 <----> uav1 <----> uav2 <----> warty2
    #
    # flows: warty1 -> warty2, warty2 -> warty1

    r_fast, v_fast = 4000, 1000
    r_fine, v_fine = 1500, 700
    r_slow, v_slow = 500, 300

    rate_graph = RateGraph(0, [])
    rate_graph.edges += [RateEdge(0, 'warty1', n, r, v) for n, r, v in zip(['uav1', 'uav2', 'warty2'], [r_fast, r_fine, r_slow], [v_fast, v_fine, v_slow])]
    rate_graph.edges += [RateEdge(0, 'uav1', n, r, v) for n, r, v in zip(['warty1', 'uav2', 'warty2'], [r_fast, r_fast, r_fine], [v_fast, v_fast, v_fine])]
    rate_graph.edges += [RateEdge(0, 'uav2', n, r, v) for n, r, v in zip(['warty2', 'uav1', 'warty1'], [r_fast, r_fast, r_fine], [v_fast, v_fast, v_fine])]
    rate_graph.edges += [RateEdge(0, 'warty2', n, r, v) for n, r, v in zip(['uav2', 'uav1', 'warty1'], [r_fast, r_fine, r_slow], [v_fast, v_fine, v_slow])]

    comm_spec = CommSpec()
    comm_spec.flows =  [FlowSpec('warty1', 'warty2', 1000, 0.9)]
    comm_spec.flows += [FlowSpec('warty2', 'warty1', 1000, 0.8)]
    comm_spec.flows += [FlowSpec('uav2', 'uav1', 600, 0.7)]

    name2idx, idx2name, rate_mean, rate_var = parse_rate_graph(rate_graph)

    slack, routes, status = socp.solve(comm_spec.flows, rate_mean, rate_var, name2idx)
    if status != 'optimal':
        print('robust routing problem returned with status: {}'.format(status))
        return

    socp_info(routes, comm_spec.flows, idx2name, rate_mean, rate_var)


def warthog_parade_test():

    rospy.init_node('robust_routing_test')
    comm_spec_pub = rospy.Publisher('robust_routing/comm_spec', CommSpec, queue_size=1)

    comm_spec = CommSpec()
    comm_spec.flows = [FlowSpec('control_station', f'warty{i}/radio', 10000, 0.8) for i in [1, 2, 3, 4, 5, 6]]

    rate = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        comm_spec_pub.publish(comm_spec)
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'simple_test':
        simple_routing_test()
    else:
        print('running warthog_parade_test()')
        warthog_parade_test()
