#!/usr/bin/env python

from connectivity_planner.channel_model import PiecewisePathLossModel, PathLossModel, LinearModel
import matplotlib.pyplot as plt
import matplotlib as mpl
from math import pi
import numpy as np
import argparse

# helps the figures to be readable on hidpi screens
mpl.rcParams['figure.dpi'] = 150


def derivative_test():

    cm=PiecewisePathLossModel(print_values=False)

    pts = 100
    xi = np.asarray([0, 0])
    xj = np.zeros((pts,2))
    xj[:,1] = np.linspace(0.01, 30.0, num=pts)
    dist = np.linalg.norm(xj, axis=1)

    rate = np.zeros((pts,))
    for i in range(pts):
        rate[i], _ = cm.predict_link(xi, xj[i,:])

    idx = np.random.randint(0,pts)
    step = 20
    start_idx = max(idx-20, 0)
    end_idx = min(idx+20, pts)
    Rxixj, _ = cm.predict_link(xi, xj[idx,:])
    dRdxi = cm.derivative(xi, xj[idx,:])
    dRdxj = cm.derivative(xj[idx,:], xi)
    Rtaylor = Rxixj + np.matmul(dRdxi.T, (xi - xi)) \
        + np.matmul(dRdxj.T, (xj[start_idx:end_idx,:] - xj[idx,:]).T)

    fig, ax = plt.subplots()
    ax.plot(dist, rate, 'r', linewidth=2)
    ax.plot(dist[start_idx:end_idx], np.reshape(Rtaylor, (Rtaylor.size,)), 'b', linewidth=2)
    ax.plot(dist[idx], Rxixj, 'bo', markersize=8, fillstyle='none', mew=2)
    plt.show()


def channel_plot():
    pwcm = PiecewisePathLossModel(print_values=False)
    lcm = LinearModel(print_values=False)
    cm = PathLossModel(print_values=False)

    x = np.linspace(0.1, 40.0, num=100)
    pw_rate = np.zeros(x.shape)
    cm_rate = np.zeros(x.shape)
    lm_rate = np.zeros(x.shape)
    xi = np.asarray([0.0, 0.0])
    for i in range(x.shape[0]):
        xj = np.asarray([0.0, x[i]])
        pw_rate[i], _ = pwcm.predict_link(xi, xj)
        lm_rate[i], _ = lcm.predict_link(xi, xj)
        cm_rate[i], _ = cm.predict_link(xi,xj)

    fig, ax = plt.subplots()
    ax.plot(x, cm_rate, 'r', lw=2, label='Log Normal')
    ax.plot(x, pw_rate, 'b--', lw=2, label='PW Log Normal')
    ax.plot(x, lm_rate, 'g-.', lw=2, label='Linear')
    ax.set_xlabel('distance (m)', fontsize='xx-large')
    ax.set_ylabel('Normalized Rate', fontsize='xx-large')
    ax.legend(prop={'size': 16})
    plt.show()


def power_plot():
    pwr1 = -53.0
    pwr2 = -52.0
    pwr3 = -51.0
    cm1 = PiecewisePathLossModel(print_values=False, l0=pwr1)
    cm2 = PiecewisePathLossModel(print_values=False, l0=pwr2)
    cm3 = PiecewisePathLossModel(print_values=False, l0=pwr3)

    x = np.linspace(0.1, 40.0, num=100)
    rate1 = np.zeros(x.shape)
    rate2 = np.zeros(x.shape)
    rate3 = np.zeros(x.shape)
    xi = np.asarray([0.0, 0.0])
    for i in range(x.shape[0]):
        xj = np.asarray([0.0, x[i]])
        rate1[i], _ = cm1.predict_link(xi, xj)
        rate2[i], _ = cm2.predict_link(xi,xj)
        rate3[i], _ = cm3.predict_link(xi, xj)

    fig, ax = plt.subplots()
    ax.plot(x, rate1, 'b--', lw=2, label=f'{pwr1} dBm')
    ax.plot(x, rate2, 'r', lw=2, label=f'{pwr2} dBm')
    ax.plot(x, rate3, 'g-.', lw=2, label=f'{pwr3} dBm')
    ax.set_xlabel('distance (m)', fontsize='xx-large')
    ax.set_ylabel('Normalized Rate', fontsize='xx-large')
    ax.legend(prop={'size': 16})
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('test', type=str, help='test to run',
                        choices=['derivative', 'plot', 'power'])

    args = parser.parse_args()

    if args.test == 'derivative':
        print('running derivative_test()')
        derivative_test()
    elif args.test == 'plot':
        print('running channel_plot()')
        channel_plot()
    elif args.test == 'power':
        print('running power_plot()')
        power_plot()
