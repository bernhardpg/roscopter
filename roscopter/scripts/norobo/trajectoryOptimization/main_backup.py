from pydrake.all import (MathematicalProgram, OsqpSolver, SnoptSolver, PiecewisePolynomial, eq, ge, le)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pydrake.solvers import branch_and_bound
from numpy import dot
from scipy.special import erfinv
from math import erf, sqrt


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Obstacle:
    def __init__(self, h, b):
        self.h = h
        self.b = b


class BoxObsticle:
    def __init__(self, _board_max_pos, p1, p2, p3, p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4

        h1 = np.array([0, 1])
        h2 = np.array([1, 0])
        h3 = np.array([0, -1])
        h4 = np.array([-1, 0])

        b1 = np.array([_board_max_pos, p1.y])
        b2 = np.array([p1.x, _board_max_pos])
        b3 = np.array([_board_max_pos, -p4.y])
        b4 = np.array([-p2.x, _board_max_pos])

        self.obstacles = [Obstacle(h1, b1), Obstacle(h2, b2), Obstacle(h3, b3), Obstacle(h4, b4)]

    def getHeight(self):
        return abs(self.p3.y - self.p1.y)

    def getWidth(self):
        return abs(self.p2.x - self.p1.x)

    def getObstacles(self):
        return self.obstacles


def get_residual(state, next_state, thrust, dt):
    global A, b
    state_dot = A.dot(state.T) + dt * b.dot(thrust.T)
    residual = next_state - state_dot
    return residual


def get_big_M(goal, start):
    # big-M parameter for the horizontal axis
    M = abs(np.array([goal[0] - start[0], goal[1] - start[1]]))
    return np.array([10, 10])


def setDynamics(prog, state, thrust, dt):
    global time_steps
    for i in range(time_steps):
        residuals = get_residual(state[i], state[i + 1], thrust[i], dt)
        for res in residuals:
            prog.AddLinearConstraint(res == 0)


def getCovMatrix(initial_cov):
    cov_matrix = np.zeros(time_steps * 2 * 2).reshape(time_steps, 2, 2)
    cov_matrix[0] = initial_cov

    for i in range(time_steps - 1):
        cov_matrix[i + 1] = dot(dot(A, cov_matrix[i]), A.T) + initial_cov
    return cov_matrix


def setCollisionConstraints(prog, state, active_constraints, objects, cov_matrix, deltas):
    global time_steps, start_pos, end_pos
    for (_, object) in enumerate(objects):
        for t in range(time_steps):
            for (o, obstacle) in enumerate(object.getObstacles()):
                v = sqrt(2 * obstacle.h.T.dot(cov_matrix[t]).dot(obstacle.h)) * erfinv(1 - 2 * deltas[t, o])
                print("o = {}".format(obstacle.h.T.dot(state[t])))
                prog.AddLinearConstraint(
                    le(obstacle.h * state[t],
                       obstacle.b - v + (1 - active_constraints[t, o]) * get_big_M(start_pos, end_pos)))


def addNormalConstraints(_prog, _thrust, _active_constraints):
    global end_pos

    prog.AddLinearConstraint(eq(state[0], start_pos))
    prog.AddLinearConstraint(eq(state[-1], end_pos))

    for i in range(time_steps):
        _prog.AddLinearConstraint(_thrust[i, 0] <= 1)
        _prog.AddLinearConstraint(_thrust[i, 1] <= 1)
        _prog.AddLinearConstraint(_thrust[i, 0] >= -1)
        _prog.AddLinearConstraint(_thrust[i, 1] >= -1)

    for i in range(time_steps):
        _prog.AddCost((state[i] - end_pos).dot(state[i] - end_pos))

    for i in range(time_steps):
        _prog.AddLinearConstraint(np.sum(_active_constraints[i]) == 1)


def probability_of_linear_constraint_holding(h, g, xbar, Sigma_x):
    prob = 0.5 + 0.5 * erf((g - dot(h.T, xbar)) / (sqrt(2 * dot(h.T, dot(Sigma_x, h)))))
    return prob


def is_linear_chance_constraint_active(x_bar, sigma_x, h, g, delta, eta=0.001):
    prob = probability_of_linear_constraint_holding(h, g, x_bar, sigma_x)
    return abs(prob - 1 + delta) <= eta


def get_active_constraint_for_object(constraints, object):
    active_constraints = []
    for t, o in constraints:
        if o == object:
            active_constraints.append(o)
    return active_constraints


time_steps = 5
dt = 2
thrust_dim = 2
alpha = 0.8
state_dim = 2
end_pos = np.array([7, 7])
start_pos = np.array([7, 1])
board_max_pos = 10
A = np.array([[1, 0],
              [0, 1]])
b = np.array([[1, 0],
              [0, 1]])
initial_sigma = np.array([[0.01, 0],
                          [0, 0.01]])
obstacles = [BoxObsticle(board_max_pos, Point(3, 3), Point(5, 3), Point(3, 5), Point(5, 5))]

cov_matrix = getCovMatrix(initial_sigma)

total_delta = 0.1  # TODO: Change this to individual constraint.

deltas = np.zeros(4 * len(obstacles) * time_steps)
for i in range(len(deltas)):
    deltas[i] = (4 * total_delta) / len(deltas)
deltas = deltas.reshape(time_steps, 4 * len(obstacles))
currentLoss = np.inf

iteration_counter = 0
while True:
    prog = MathematicalProgram()

    state = prog.NewContinuousVariables(time_steps + 1, state_dim, 'state')
    thrust = prog.NewContinuousVariables(time_steps + 1, thrust_dim, 'thrust')
    active_constraints = prog.NewBinaryVariables(time_steps + 1, 4 * len(obstacles), 'thrust')

    setDynamics(prog, state, thrust, dt)
    setCollisionConstraints(prog, state, active_constraints, obstacles, cov_matrix, deltas)
    addNormalConstraints(prog, thrust, active_constraints)

    bb = branch_and_bound.MixedIntegerBranchAndBound(prog, OsqpSolver().solver_id())
    result = bb.Solve()

    state_opt = bb.GetSolution(state)
    active_contraint_opt = bb.GetSolution(active_constraints)

    # %%
    current_active_constraints = set()
    for t in range(time_steps):
        for x_bar in state_opt:
            for object in obstacles:
                for (o, constraint) in enumerate(object.getObstacles()):
                    the_sum = 2 * dot(constraint.h[0], dot(cov_matrix[t, 0], constraint.h[0].T))
                    for n in range(2):
                        active1 = is_linear_chance_constraint_active(x_bar[n], cov_matrix[t, n, n], constraint.h[n],
                                                                     constraint.b[n], deltas[t, o], eta=0.001)
                        if active1:
                            print("Found active constraint, t = {}, i = {}, x_bar = {}".format(t, o, x_bar))
                            current_active_constraints.add((t, o))

    if len(current_active_constraints) == 0 or len(current_active_constraints) == 4:
        print("no more constraints, len = {}".format(len(current_active_constraints)))
        break

    for o in range(len(obstacles[0].getObstacles())):  # Warning: assume just one obstacle
        obstacle = obstacles[0].getObstacles()[o]
        for t in range(time_steps):
            if (t, o) not in current_active_constraints:
                for n in range(2):
                    prob = probability_of_linear_constraint_holding(obstacle.h[n], obstacle.b[n], state_opt[t, n],
                                                                    cov_matrix[t, n, n])
                    active = active_contraint_opt[t, o] > 0.1
                    if active:
                        previous_delta = deltas[t, o]
                        deltas[t, o] = alpha * deltas[t, o] + (1 - alpha) * (1 - prob)
                        print("previous_delta = {}, new delta = {}".format(previous_delta, deltas[t, o]))
                        if deltas[t, o] > 0.19:
                            print("Error")

    for (t, o) in current_active_constraints:
        added_risk = sum(deltas[:, o]) / len(get_active_constraint_for_object(current_active_constraints, o))
        old_delta = deltas[t, o]
        deltas[t, o] = deltas[t, o] + added_risk
        print("old Delta = {}, new delta = {}".format(old_delta, deltas[t, o]))

    # residual = total_delta - sum(deltas)
    #
    # for activeConst in current_active_constraints:
    #     deltas[activeConst] = residual/len(current_active_constraints)
    break
    # iteration_counter += 1
    # if iteration_counter == 3:
    #     break

# %%
plt.figure()

for obstacle in obstacles:
    currentAxis = plt.gca()
    currentAxis.add_patch(
        Rectangle((obstacle.p1.x, obstacle.p1.y), obstacle.getWidth(), obstacle.getHeight(), facecolor="grey"))

# print(decision_variables_opt)

plt.plot(state_opt.T[0], state_opt.T[1])
plt.show()
print(state_opt)
print("----")
print(active_contraint_opt)