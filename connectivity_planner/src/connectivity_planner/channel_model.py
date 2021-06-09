import numpy as np
from scipy import special, spatial
from scipy.optimize import toms748


def dbm2mw(dbm):
    """Convert between decible-milliwatts (dBm) and milliwatts."""
    return 10.0 ** (np.asarray(dbm) / 10.0)


class PathLossModel:
    """A distance dependent channel model with log normal fading.

    This class implements the distance dependent path loss with log-normal
    fading channel model from [1]. See section 4.1 and M1 in section 4.3 for
    details.

    [1] Fink, Jonathan. "Communication for teams of networked robots." PhD
    Thesis (2011).

    """

    def __init__(self, print_values=True, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0):
        self.L0 = l0                # transmit power (dBm)
        self.n = n                  # decay rate
        self.N0 = n0                # noise at receiver (dBm)
        self.a = a                  # sigmoid parameter 1
        self.b = b                  # sigmoid parameter 2
        self.PL0 = dbm2mw(self.L0)  # transmit power (mW)
        self.PN0 = dbm2mw(self.N0)  # noise at receiver (mW)
        if print_values is True:
            print('L0 = %.3f' % self.L0)
            print('n  = %.3f' % self.n)
            print('N0 = %.3f' % self.N0)
            print('a  = %.3f' % self.a)
            print('b  = %.3f' % self.b)

    def predict(self, x):
        """Compute the expected channel rate and variance for a team of agents.

        Inputs:
          x: a Nx2 list of node positions [x y]

        Outputs:
          rate: matrix of expected channel rates between each pair of agents
          var: matrix of channel rate variances between each pair of agents

        """
        d = spatial.distance_matrix(x, x)
        dist_mask = ~np.eye(d.shape[0], dtype=bool)
        power = np.zeros(d.shape)
        power[dist_mask] = dbm2mw(self.L0 - 10 * self.n * np.log10(d[dist_mask]))
        rate = special.erf(np.sqrt(power / self.PN0))
        var = (self.a * d / (self.b + d)) ** 2
        return rate, var

    def predict_link(self, xi, xj):
        """Compute the expected channel rate and variance of a single link.

        Inputs:
          xi: 1x2 node position
          xj: 1x2 node position

        Outputs:
          rate: expected channel rate between xi, xj
          var: variance ("confidence") in expected channel rate between xi, xj

        """
        d = np.linalg.norm(xi - xj)
        power = dbm2mw(self.L0 - 10 * self.n * np.log10(d))
        rate = special.erf(np.sqrt(power / self.PN0))
        var = (self.a * d / (self.b + d)) ** 2
        return rate, var

    def derivative_coeff(self, d):
        return - 10.0**(self.L0/20) * self.n * np.sqrt(d ** (-self.n) / self.PN0) \
            * np.exp(-10.0**(self.L0 / 10.0) * d**(-self.n) / self.PN0) \
            / (np.sqrt(np.pi) * d) / d

    def derivative(self, xi, xj):
        """Compute the derivative of the channel rate function w.r.t xi.

        Note: the derivative of the channel with respect to xj can be found by
        swapping the inputs (i.e.: derivative(xj, xi))

        Inputs:
          xi: [x, y] node position
          xj: [x, y] node position

        Outputs:
          der: 2x1 derivative of Rij w.r.t xi

        """
        xi = np.reshape(xi, (2,1))
        xj = np.reshape(xj, (2,1))

        dist = np.linalg.norm(xi - xj)
        if dist < 1e-6:
            return np.zeros((2,1))
        return self.derivative_coeff(dist) * (xi - xj)

    def calculate_range(self, max_range=100):
        return toms748(lambda a : self.predict_link(np.zeros((2,)), np.asarray([a,0]))[0]-1e-10, 1, max_range)


class PiecewisePathLossModel(PathLossModel):
    """A piecewise distance dependent channel model with log normal fading.

    This class combines the distance dependent path loss with log-normal fading
    channel model from [1] (see section 4.1 and M1 in section 4.3 for details)
    with a linear model at long distances that ensures the predicted rate goes
    to zero as the distance gets large. Note that M1 by itself never quite goes
    to zero even as the distance grows well beyond the distance two agents can
    communicate; hence the need for a linear section at long distances that
    ensures this desired behavior.

    [1] Fink, Jonathan. "Communication for teams of networked robots." PhD
    Thesis (2011).

    """

    def __init__(self, print_values=True, n0=-70.0, n=2.52, l0=-53.0, a=0.2, b=6.0,
                 transition_rate=0.2320007930054694):
        PathLossModel.__init__(self, print_values, n0, n, l0, a, b)

        self.trans_dist = (self.PL0 / (self.PN0 * special.erfinv(transition_rate) ** 2)) ** (1/self.n)

        # find the slope of R(xi, xj) at the cuttoff distance; this will serve
        # as the slope of the linear section that decays to zero
        dRdd = PathLossModel.derivative(self, np.asarray([self.trans_dist, 0.0]), np.zeros((2,)))
        self.m = dRdd[0].item() # by construction dR/dy = 0; dR/dx = slope (change in distance)

        # find the y-intercept of the linear portion
        self.b = transition_rate - self.m * self.trans_dist

        self.cutoff_dist = - self.b / self.m # when the rate drops to zero

    def predict(self, x):
        """Compute the expected channel rate and variance for a team of agents.

        Inputs:
          x: a Nx2 list of node positions [x y]

        Outputs:
          rate: matrix of expected channel rates between each pair of agents
          var: matrix of channel rate variances between each pair of agents

        """
        rate, var = PathLossModel.predict(self, x)

        edm = spatial.distance_matrix(x, x)
        dist_mask =  edm > self.trans_dist
        rate[dist_mask] = np.maximum(self.m * edm[dist_mask] + self.b, np.zeros(edm[dist_mask].size))

        # TODO zero out variance when rate is zero?

        return rate, var

    def predict_link(self, xi, xj):
        """Compute the expected channel rate and variance of a single link.

        Inputs:
          xi: 1x2 node position
          xj: 1x2 node position

        Outputs:
          rate: expected channel rate between xi, xj
          var: variance ("confidence") in expected channel rate between xi, xj

        """
        rate, var = PathLossModel.predict_link(self, xi, xj)

        dist = np.linalg.norm(xi - xj)
        if dist > self.trans_dist:
            rate = max(self.m * dist + self.b, 0.0)

        # TODO zero out variance when rate hits zero?
        # if rate < 0.0:
        #     var = 0.0

        return rate, var

    def derivative(self, xi, xj):
        """Compute the derivative of the channel rate function w.r.t xi.

        Note: the derivative of the channel with respect to xj can be found by
        swapping the inputs (i.e.: derivative(xj, xi))

        Inputs:
          xi: [x, y] node position
          xj: [x, y] node position

        Outputs:
          der: 2x1 derivative of Rij w.r.t xi

        """
        diff = xi - xj
        dist = np.linalg.norm(diff)

        if dist > self.cutoff_dist:
            return np.zeros((2,1))
        elif dist > self.trans_dist:
            return self.m / dist * diff
        else:
            return PathLossModel.derivative(self, xi, xj)


class LinearModel:
    """A channel mode that decays linearly with distance."""

    def __init__(self, print_values=False, max_range=30.0):
        self.max_range = max_range
        if print_values is True:
            print('max_range = %.3f' % self.max_range)

    def predict(self, x):
        """Compute the expected channel rate and variance for a team of agents.

        Inputs:
          x: a Nx2 list of node positions [x y]

        Outputs:
          rate: matrix of expected channel rates between each pair of agents
          var: matrix of channel rate variances between each pair of agents

        """
        dist = spatial.distance_matrix(x, x)
        rate = np.maximum(-1.0/self.max_range * dist + 1.0, 0.0)
        rate[np.eye(dist.shape[0], dtype=bool)] = 0.0
        var = np.zeros(dist.shape)
        return rate, var

    def predict_link(self, xi, xj):
        """Compute the expected channel rate and variance of a single link.

        Inputs:
          xi: 1x2 node position
          xj: 1x2 node position

        Outputs:
          rate: expected channel rate between xi, xj
          var: variance ("confidence") in expected channel rate between xi, xj

        """
        rate = max(-1.0/self.max_range * np.linalg.norm(xi - xj) + 1.0, 0.0)
        var = 0.0
        return rate, var

    def derivative(self, xi, xj):
        """Compute the derivative of the channel rate function w.r.t xi.

        Note: the derivative of the channel with respect to xj can be found by
        swapping the inputs (i.e.: derivative(xj, xi))

        Inputs:
          xi: [x, y] node position
          xj: [x, y] node position

        Outputs:
          der: 2x1 derivative of Rij w.r.t xi

        """
        xi = np.reshape(xi, (2,1))
        xj = np.reshape(xj, (2,1))

        diff = xi - xj
        dist = np.linalg.norm(xi - xj)
        if dist > self.max_range:
            return np.zeros((2,1))
        return -1.0 / self.max_range * diff / dist
