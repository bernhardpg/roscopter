from constants import *
from helpers import *
from pydrake.all import (LinearQuadraticRegulator)


class LQRController:  # TODO: Make uncoupled from constants.
    def __init__(self):
        self.A, self.B, self.Q, self.R = self.getABQR()
        self.K, self.S = LinearQuadraticRegulator(self.A, self.B, self.Q, self.R)

    def getU(self, current_state, desired_state):
        u = -self.K.dot(current_state.get_formated_state() - desired_state)
        # u[0] /= (2 * drone_mass * g)
        u[0] /= 59.844
        u[0] += 0.5

        return np.array([u[1], u[2], u[3], u[0]])

    def getABQR(self):
        A_elements = [(3, 1), (4, 1), (5, 1),
                      (0, 0), (0, 0), (0, 0),
                      (1, -g), (0, g), (0, 0),
                      (6, 1), (7, 1), (8, 1)]

        B_elements = [(0, 0), (0, 0), (0, 0),
                      (1, 1 / I_x), (2, 1 / I_y), (3, 1 / I_z),
                      (0, 0), (0, 0), (0, 1 / drone_mass),
                      (0, 0), (0, 0), (0, 0)]

        A = convert_tuple_to_array(A_elements, drone_n_states, drone_n_states)  # TODO: possible to make this array in a better way?
        B = convert_tuple_to_array(B_elements, drone_n_states, drone_n_control)
        Q = get_identity_matrix([10, 10, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10], drone_n_states)
        R = np.identity(drone_n_control)

        return A, B, Q, R