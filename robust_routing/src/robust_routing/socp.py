import cvxpy as cp
import numpy as np
from scipy import stats


def solve(flows, rate_mean, rate_var, name2idx):
    """Solve robust routing problem for the given network state and comm. reqs.

    Inputs:
      flows: a list of FlowSpec messages
      rate_mean: an NxN matrix of channel rates
      rate_var: an NxN matrix of channel rate variances
      name2idx: maps agent names to indices used in the optimization

    Outputs:
      slack: slack of the associated robust routing solution
      routes: optimal routing variables
      status: 'optimal' if a soln was found

    """
    N = rate_mean.shape[0]  # number of agents
    P = len(flows)          # number of data flows

    # socp constraint coefficient matrices
    a_mat, b_mat, zero_vars, conf, m_ik = cone_constraints(flows, rate_mean, rate_var, name2idx)

    # optimization variables
    slack, routes = cp.Variable((1)), cp.Variable((N*N*P))
    y = cp.hstack([routes, slack])

    # 2nd order cone constraints
    cone_consts = []
    for i in range(a_mat.shape[0]):
        cone_consts += [cp.SOC((cp.matmul(b_mat[i,:], y) - m_ik[i]) / conf[i], cp.matmul(cp.diag(a_mat[i,:]), y))]

    # linear availability constraints
    routing_sum_mat = cp.reshape(routes[:N*N], (N,N))  # acts like np.reshape with 'F'
    for k in range(1, P):  # summing over dim 3 manually b.c. cvxpy only supports 2D matrices
        routing_sum_mat += cp.reshape(routes[k*N*N:(k+1)*N*N], (N,N))
    lin_consts = [cp.sum(routing_sum_mat, 2) <= 1]
    lin_consts += [cp.sum(routing_sum_mat, 1) <= 1]

    # sign and equality constraints
    sign_consts = [0 <= routes, routes <= 1, 0 <= slack]
    zero_var_consts = [routes[np.reshape(zero_vars, (N*N*P), 'F')] == 0]

    # solve program with CVX
    constraints = cone_consts + lin_consts + sign_consts + zero_var_consts
    socp = cp.Problem(cp.Maximize(slack), constraints)
    socp.solve()

    if socp.status == 'optimal':
        routing_vars = np.reshape(routes.value, (N,N,P), 'F')
        return slack.value[0], routing_vars, socp.status

    return None, None, socp.status


def cone_constraints(flows, rate_mean, rate_var, name2idx):
    """Compute constraint coefficient matrices for the robust routing problem.

    2nd order cone constraints of the form:
    ||A*y + b|| <= c^T*y + d
    are equivalent to the node margin constraints:
    (B*y - m_ik) / ||A*y|| >= I(eps)
    where y is the vector of optimization variables and I(eps) is the
    inverse normal cumulative distribution function.

    Inputs:
      flows: a list of FlowSpec messages defining communication constraints
      rate_mean: an NxN matrix of link rates
      rate_var: an NxN matrix of link rate variances
      name2idx: mapping between names in flows and 0-indexed vectors, matrices

    Outputs:
      a_mat: variance coefficient matrix
      b_mat: mean coefficient matrix
      zero_vars: which optimization variables can safely be set to zero
      conf: the probabilistic confidence of each constraint
      m_ik: the required rate margin of each constraint

    """
    N = rate_mean.shape[0]  # number of agents
    P = len(flows)          # number of data flows

    # variables that should be zero
    zero_vars = np.reshape(np.eye(N, dtype=bool), (N,N,1), 'F')
    zero_vars = np.repeat(zero_vars, P, axis=2)
    for k in range(P):
        zero_vars[:, name2idx[flows[k].tx], k] = True

    # node margin constraints

    a_mat = np.zeros(((N-1)*P, N*N*P+1))
    b_mat = np.zeros(((N-1)*P, N*N*P+1))
    m_ik = np.zeros(((N-1)*P))

    idx = 0
    for k in range(P):
        for i in range(N):
            if i == name2idx[flows[k].rx]:
                continue

            if i == name2idx[flows[k].tx]:
                m_ik[idx] = flows[k].rate

            aki = np.zeros((N,N))
            bki = np.zeros((N,N))

            aki[:,i] = np.sqrt(rate_var[:,i])  # incoming
            aki[i,:] = np.sqrt(rate_var[i,:])  # outgoing
            aki[zero_vars[:,:,k]] = 0.0

            bki[:,i] = -rate_mean[:,i]  # incoming
            bki[i,:] =  rate_mean[i,:]  # outgoing
            bki[zero_vars[:,:,k]] = 0.0

            a_mat[idx, k*N*N:(k+1)*N*N] = np.reshape(aki, (1,-1), 'F')
            b_mat[idx, k*N*N:(k+1)*N*N] = np.reshape(bki, (1,-1), 'F')
            b_mat[idx, N*N*P] = -1

            idx += 1

    # probabilistic confidence requirements
    conf = stats.norm.ppf(np.repeat([f.confidence for f in flows], N-1))

    return a_mat, b_mat, zero_vars, conf, m_ik
