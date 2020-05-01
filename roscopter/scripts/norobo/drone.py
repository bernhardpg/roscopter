import numpy as np
from pydrake import symbolic
from constants import *
from helpers import *


class Drone:
    def __init__(self, state, m, ixx, iyy, izz):  # State of the drone
        self.state = state
        self.m = m
        self.ixx = ixx
        self.iyy = iyy
        self.izz = izz

    def getDrakeDelta(self, state, t_phi, t_theta, t_gamma, f):
        m, ixx, iyy, izz = self.m, self.ixx, self.iyy, self.izz
        phi, theta, gamma = state[3:6]
        u, v, w = state[6:9]
        p, q, r = state[9:12]

        def sin(angle):  # sin and cos are defined to make math expressions cleaner.
            return np.sin(angle)
            # return symbolic.sin(angle)

        def cos(angle):
            return np.cos(angle)
            # return symbolic.cos(angle)

        # dx, dy, dz = rotation_matrix(phi, theta, gamma).dot(np.array([u, v, w]))
        # du, dv, dw = np.array([r*v-q*w, p*w-r*u, p*u-p*v]) + np.array([-g*sin(theta), g*cos(theta)*sin(gamma), g*cos(theta)*cos(gamma)]) + 1/m*np.array([0, 0, -f])
        # dphi, dtheta, dgamma = transformation_matrix(phi, theta).dot(np.array([p, q, r]))
        # dp, dq, dr = np.array([((iyy-izz)/ixx)*q*r, ((izz-ixx)/iyy)*p*r, ((ixx-iyy)/izz)*p*q]) + np.array([(1/ixx)*t_phi, (1/iyy)*t_theta, (1/izz)*t_gamma])

        # dx, dy, dz = rotation_matrix(phi, theta, gamma).dot(np.array([u, v, w]))
        # du, dv, dw = np.array([r*v-q*w, p*w-r*u, p*u-p*v]) + np.array([-g*sin(theta), g*cos(theta)*sin(gamma), g*cos(theta)*cos(gamma)]) + 1/m*np.array([0, 0, -f])
        # dphi, dtheta, dgamma = np.array([p, q, r])
        # dp, dq, dr = np.array([(1/ixx)*t_phi, (1/iyy)*t_theta, (1/izz)*t_gamma])


        dx, dy, dz = np.array([u, v, w])
        du, dv, dw =   1/m*np.array([0, 0, -f])
        dphi, dtheta, dgamma = np.array([p, q, r])
        dp, dq, dr = np.array([(1/ixx)*t_phi, (1/iyy)*t_theta, (1/izz)*t_gamma])

        return np.array([dx, dy, dz, dphi, dtheta, dgamma, du, dv, dw,  dp, dq, dr])

    def discrete_dynamics(self, state, next_state, thrust, time_step):
        state_delta = self.getDrakeDelta(next_state, thrust[0], thrust[1], thrust[2], thrust[3])

        residuals = next_state - state - time_step * state_delta
        return residuals