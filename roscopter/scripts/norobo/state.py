from enum import Enum
import numpy as np


class RunState(Enum):
    MIT = 1
    KTH_QUARTERNION = 2
    KTH_EULER = 3
    KTH_NED = 4


class State:
    initiated = False

    inertial_angle = None
    inertial_position = None
    inertial_velocity = None
    inertial_angle_velocity = None

    body_position = None
    body_angle = None
    body_velocity = None
    body_angle_velocity = None

    def get_formated_state(self):
        phi, theta, gamma = self.inertial_angle.phi, self.inertial_angle.theta, self.inertial_angle.gamma
        x, y, z = self.inertial_position.x, self.inertial_position.y, self.inertial_position.z
        p, q, r = self.body_angle_velocity.p, self.body_angle_velocity.q, self.body_angle_velocity.r
        u, v, w = self.body_velocity.u, self.body_velocity.v, self.body_velocity.w
        return np.array([phi, theta, gamma, p, q, r, u, v, -w, x, y, -z])


class Angle:
    def __init__(self, phi, theta, gamma):
        self.phi = phi
        self.theta = theta
        self.gamma = gamma


class AngleVelocity:
    def __init__(self, p, q, r):
        self.p = p
        self.q = q
        self.r = r


class Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Velocity:
    def __init__(self, u, v, w):
        self.u = u
        self.v = v
        self.w = w