import numpy as np
from math import floor, log10


def parse_rate_graph(rate_graph_msg):
    """Extract names, rates, and variances from ROS RateGraph message.

    Input:
      rate_graph_msg: ROS RateGraph message to parse

    Output:
      name2idx: maps agent names to indices used in the optimization
      idx2name: inverse of name2idx
      rate_mean: an NxN matrix of channel rates
      rate_var: an NxN matrix of channel rate variances

    """
    name2idx = {}
    for edge in rate_graph_msg.edges:
        if edge.tx not in name2idx.keys():
            name2idx[edge.tx] = len(name2idx)
        if edge.rx not in name2idx.keys():
            name2idx[edge.rx] = len(name2idx)
    idx2name = {name2idx[key]: key for key in name2idx.keys()}

    N = len(name2idx)
    rate_mean = np.zeros((N,N))
    rate_var = np.zeros((N,N))
    for edge in rate_graph_msg.edges:
        tx_idx, rx_idx = name2idx[edge.tx], name2idx[edge.rx]
        rate_mean[tx_idx, rx_idx] = edge.rate
        rate_var[tx_idx, rx_idx] = edge.rate_variance

    return name2idx, idx2name, rate_mean, rate_var


def fwstr(string, width):
    """Print a right justified fixed width string padded with spaces."""
    return (width-len(string))*' ' + string


def socp_info(routes, flows, idx2name, rate_mean=None, rate_var=None, threshold=1e-5):
    """Print information about the robust routing solution to the console.

    Input:
      routes: an NxNxK array of routing variables
      flows: an array of FlowSpec requirements with length(flows) == K
      idx2name: maps indices used in the optimization to agent names
      rate_mean: an NxN matrix of channel rates
      rate_var: an NxN matrix of channel rate variances
      threshold: all routes below threshold are zeroed out

    """
    N = routes.shape[0]
    K = routes.shape[2]

    pad = 2
    tab = 2
    num_str = '{:.' + str(abs(floor(log10(threshold)))+1) + 'f}'

    max_name_width = np.max([len(val) for val in list(idx2name.values())])
    max_val_width = len(num_str.format(np.max(routes)))
    cell_width = max(max_name_width, max_val_width) + pad
    node_names = [fwstr(idx2name[i], cell_width) for i in range(N)]

    for k, flow in zip(range(K), flows):

        aij = routes[...,k]

        # flow info
        flow_info = 'Flow {}: {} -> {}, rate = '.format(k+1, flow.tx, flow.rx)
        flow_info += (num_str + ', confidence = ' + num_str + ':').format(flow.rate, flow.confidence)
        print(flow_info)

        # table header
        print((tab + cell_width + 1)*' ' + ''.join(node_names) + ' ' + fwstr('Tx', cell_width))

        # table separator
        print((tab + cell_width)*' ' + (N*cell_width+2)*'-')

        # routes rows
        for i in range(N):
            vals_strs = [num_str.format(val) if val > threshold else '-' for val in aij[i,:]]
            vals_str = ''.join([fwstr(v, cell_width) for v in vals_strs])
            tx = num_str.format(np.sum(aij[i,:])) if np.max(aij[i,:]) > threshold else '-'
            print(tab*' ' + node_names[i] + '|' + vals_str + '|' + fwstr(tx, cell_width))

        # table separator
        print((tab + cell_width)*' ' + (N*cell_width+2)*'-')

        # rx row
        rx_strs = [num_str.format(np.sum(aij[:,j])) if np.max(aij[:,j]) > threshold else '-' for j in range(N)]
        rx_str = ''.join([fwstr(v, cell_width) for v in rx_strs])
        print(tab*' ' + fwstr('Rx', cell_width) + '|' + rx_str + '\n')

    if rate_mean is not None:
        with np.printoptions(precision=3, suppress=True):
            print('rate_mean:')
            print(rate_mean)
            print('\n')
    if rate_var is not None:
        with np.printoptions(precision=3, suppress=True):
            print('rate_var:')
            print(rate_var)
