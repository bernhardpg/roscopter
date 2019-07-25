import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from plotWindow import plotWindow
from ekf_data import Log
import scipy.linalg


def plotLla():
    f = plt.figure()
    plt.suptitle("lla")
    for i in range(3):
        plt.subplot(3, 2,2*i+1)
        if i == 2:
            plt.plot(data.lla['t'], data.lla['hat'][:,i], label=r"$\hat{z}$")
            plt.plot(data.lla['t'], data.lla['bar'][:,i], label=r"$\bar{z}$")
        else:
            plt.plot(data.lla['t'], data.lla['hat'][:,i]*180.0/np.pi, label=r"$\hat{z}$")
            plt.plot(data.lla['t'], data.lla['bar'][:,i]*180.0/np.pi, label=r"$\bar{z}$")
        if i == 0:
            plt.legend()
    plt.subplot(1,2,2)
    plt.plot(data.lla['hat'][:,1]*180/np.pi, data.lla['hat'][:,0]*180.0/np.pi, label=r"$\hat{z}$")
    plt.plot(data.lla['bar'][:,1]*180/np.pi, data.lla['bar'][:,0]*180.0/np.pi, label=r"$\bar{z}$")
    pw.addPlot("lla", f);

def plotIMU():
    f = plt.figure()
    plt.suptitle('Imu')
    for i in range(3):
        plt.subplot(4, 2, 2*i+1)
        plt.plot(data.imu['t'], data.imu['a'][:, i], label=imu_titles[i])
        plt.legend()
        plt.subplot(4, 2, 2*i+2)
        plt.plot(data.imu['t'], data.imu['w'][:, i], label=imu_titles[i+3])
        plt.legend()
    plt.subplot(4,2,7)
    plt.plot(data.imu['t'], scipy.linalg.norm(data.imu['a'], axis=1))
    pw.addPlot("IMU", f)

def plotPosition():
    f = plt.figure()
    plt.suptitle('Position')
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.title(xtitles[i])
        plt.plot(data.ref['t'], data.ref['x']['p'][:,i], label='ref')
        plt.plot(data.x['t'], data.x['x']['p'][:,i], label=r"$\hat{x}$")
        if plotCov:
            plt.plot(data.cov['t'], data.x['x']['p'][:,i] + 2.0*np.sqrt(data.cov['P'][:, i,i]), '-k', alpha=0.3)
            plt.plot(data.cov['t'], data.x['x']['p'][:,i] - 2.0*np.sqrt(data.cov['P'][:, i,i]), '-k', alpha=0.3)
        if i == 0:
            plt.legend()
    pw.addPlot("Position", f)

def plotVelocity():
    f = plt.figure()
    plt.suptitle('Velocity')
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.title(vtitles[i])
        plt.plot(data.x['t'], data.x['v'][:,i], label=r"$\hat{x}$")
        if plotCov:
            plt.plot(data.cov['t'], data.x['v'][:,i] + 2.0*np.sqrt(data.cov['P'][:, i+6,i+6]), '-k', alpha=0.3)
            plt.plot(data.cov['t'], data.x['v'][:,i] - 2.0*np.sqrt(data.cov['P'][:, i+6,i+6]), '-k', alpha=0.3)
        if i == 0:
            plt.legend()
    pw.addPlot("Velocity", f)

def plotAttitude():
    f = plt.figure()
    plt.suptitle('Attitude')
    for i in range(4):
        plt.subplot(4, 1, i+1)
        plt.title(xtitles[i+3])
        plt.plot(data.ref['t'], data.ref['x']['q'][:,i], label='ref')
        plt.plot(data.x['t'], data.x['x']['q'][:,i], label=r"$\hat{x}$")
        if plotCov and i > 0:
            plt.plot(data.cov['t'], data.x['x']['q'][:,i] + 2.0*np.sqrt(data.cov['P'][:, i+3,i+3]), '-k', alpha=0.3)
            plt.plot(data.cov['t'], data.x['x']['q'][:,i] - 2.0*np.sqrt(data.cov['P'][:, i+3,i+3]), '-k', alpha=0.3)
        if i == 0:
            plt.legend()
    pw.addPlot("Attitude", f)

def plotEuler():
    f = plt.figure()
    plt.suptitle('Euler')
    rad2deg = 180.0/np.pi
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.title(vtitles[i])
        plt.plot(data.ref['t'], data.ref['euler'][:,i] * rad2deg, label='ref')
        plt.plot(data.x['t'], data.x['euler'][:,i] * rad2deg, label=r"$\hat{x}$")
        if plotCov:
            plt.plot(data.cov['t'], rad2deg * (data.x['euler'][:,i] + 2.0*np.sqrt(data.cov['P'][:, i+3,i+3])), '-k', alpha=0.3)
            plt.plot(data.cov['t'], rad2deg * (data.x['euler'][:,i] - 2.0*np.sqrt(data.cov['P'][:, i+3,i+3])), '-k', alpha=0.3)
        if i == 0:
            plt.legend()
    pw.addPlot("Euler", f)


def plotImuBias():
    f = plt.figure()
    plt.suptitle('Bias')
    for i in range(3):
        for j in range(2):
            plt.subplot(3, 2, i * 2 + j + 1)
            if j == 0:
                plt.plot(data.x['t'], data.x['ba'][:, i])
                if plotCov:
                    plt.plot(data.x['t'], data.x['ba'][:, i] + 2.0*np.sqrt(data.cov['P'][:,i+9, i+9]), '-k', alpha=0.3)
                    plt.plot(data.x['t'], data.x['ba'][:, i] - 2.0*np.sqrt(data.cov['P'][:,i+9, i+9]), '-k', alpha=0.3)
            else:
                plt.plot(data.x['t'], data.x['bg'][:, i])
                if plotCov:
                    plt.plot(data.x['t'], data.x['bg'][:, i] + 2.0*np.sqrt(data.cov['P'][:,i+12, i+12]), '-k', alpha=0.3)
                    plt.plot(data.x['t'], data.x['bg'][:, i] - 2.0*np.sqrt(data.cov['P'][:,i+12, i+12]), '-k', alpha=0.3)
            plt.title(imu_titles[j * 3 + i])
        if i == 0:
            plt.legend()
    pw.addPlot("IMU Bias", f)

def plotZVRes():
    f = plt.figure()
    plt.suptitle('ZeroVel')
    for i in range(4):
        plt.subplot(4, 1, i+1)
        plt.plot(data.zvRes['t'], data.zvRes['r'][:,i], label="r")
        if i == 0:
            plt.legend()
    pw.addPlot("ZeroVel", f)

def plotGnssRes():
    f = plt.figure()
    plt.suptitle('Gnss Res')
    for i in range(3):
        for j in range(2):
            plt.subplot(3, 2, i * 2 + j + 1)
            plt.plot(data.gnssRes['t'], data.gnssRes['r'][:,j*3+i])
        if i == 0:
            plt.legend()
    pw.addPlot("Gnss Res", f)


def plotResults(directory):
    np.set_printoptions(linewidth=150)
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')

    global xtitles, vtitles, imu_titles, colors, data, pw, plotCov
    xtitles = ['$p_x$', '$p_y$', '$p_z$', '$q_w$', '$q_x$', '$q_y$', '$q_z$']
    vtitles = ['$v_x$', '$v_y$', '$v_z$']
    imu_titles = [r"$acc_x$", r"$acc_y$", r"$acc_z$", r"$\omega_x$", r"$\omega_y$", r"$\omega_z$"]
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    plotCov = True

    data = Log(directory)
    pw = plotWindow()

    plotPosition()
    plotVelocity()
    plotAttitude()
    plotEuler()
    plotLla()
    plotIMU()
    plotImuBias()

    plotZVRes()
    plotGnssRes()

    pw.show()


if __name__ == '__main__':
    import yaml
    params = yaml.load(open("../params/ekf.yaml"))
    plotResults(params["log_prefix"])
