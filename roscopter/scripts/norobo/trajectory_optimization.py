from pydrake.all import (MathematicalProgram, SnoptSolver, PiecewisePolynomial, eq)
import numpy as np
from constants import *
from drone import Drone
from pydrake.all import LinearSystem, DirectTranscription, DirectCollocation, Solve

from pydrake.examples.quadrotor import (QuadrotorPlant,
                                        StabilizingLQRController,
                                        QuadrotorGeometry)

from pydrake.geometry import SceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, VectorSystem


class TrajectoryOptimization:
    def __init__(self, time_steps=150, dt=0.1):
        self.drone = Drone(np.zeros(drone_n_states), drone_mass, I_x, I_y, I_z)
        self.time_steps = time_steps
        self.dt = dt
        self.thrust_opt = None

        # initialize optimization
        self.prog = MathematicalProgram()

        self.state_var = self.prog.NewContinuousVariables(self.time_steps + 1, 12, 'state')
        self.thrust_var = self.prog.NewContinuousVariables(self.time_steps, 4, 'thrust')
        self.set_limits(self.prog, self.state_var, self.thrust_var)

    def get_initial_guess(self, prog):
        time_limits = np.array([0, self.time_steps * self.dt])
        state_limits = np.array([0, 0])  # Possible to init array without this line ?
        for i in range(drone_n_states - 1):
            state_limits = np.vstack((state_limits, np.array([0, 0])))
        state_limits[2] = np.array([0, 1])
        state_generator = PiecewisePolynomial.FirstOrderHold(time_limits, state_limits)
        state_guess = np.vstack([state_generator.value(t * self.dt).T for t in range(self.time_steps + 1)])

        return state_guess

    def set_limits(self, prog, state, thrust):
        for t in range(self.time_steps):
            residuals = self.drone.discrete_dynamics(state[t], state[t + 1], thrust[t], self.dt)
            for residual in residuals:
                # print(residual, "\n")
                prog.AddConstraint(residual == 0)
            # break
        prog.AddLinearConstraint(eq(state[0], np.zeros(12)))
        prog.AddLinearConstraint(eq(state[-1], np.array([0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0])))

        for t in range(self.time_steps):
            prog.AddLinearConstraint(thrust[t, 3] <= drone_mass * g * 2)
            prog.AddLinearConstraint(thrust[t, 2] <= 20)
            prog.AddLinearConstraint(thrust[t, 1] <= 20)
            prog.AddLinearConstraint(thrust[t, 0] <= 20)

    def getU(self):
        if self.thrust_opt is None:
            raise ValueError("need to run optimization before getting control variables.")
        return self.thrust_opt

    def generate_path(self, state_guess=None, thrust_guess=None):
        if state_guess is not None:
            self.prog.SetInitialGuess(self.state_var, state_guess)
        if thrust_guess is not None:
            self.prog.SetInitialGuess(self.thrust_var, thrust_guess)

        solver = SnoptSolver()
        result = solver.Solve(self.prog)

        # be sure that the solution is optimal
        print("Success? {}".format(result.is_success()))

        thrust = result.GetSolution(self.thrust_var)
        self.thrust_opt = thrust
        self.state_opt = result.GetSolution(self.state_var)

        return self.state_opt, self.thrust_opt

class DrakeTrajectoryOptimization:

    def __init__(self):
        builder = DiagramBuilder()

        self.plant = builder.AddSystem(QuadrotorPlant(drone_mass, ))
        context = self.plant.CreateDefaultContext()

        N = 21
        max_dt = 0.5
        max_tf = N * max_dt
        dircol = DirectCollocation(self.plant,
                                   context,
                                   num_time_samples=N,
                                   minimum_timestep=0.05,
                                   maximum_timestep=max_dt)

        dircol.AddEqualTimeIntervalsConstraints()
        initial_state = np.zeros(12)
        dircol.AddBoundingBoxConstraint(initial_state, initial_state,
                                        dircol.initial_state())
        final_state = np.zeros(12)
        final_state[2] = 1
        dircol.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())

        result = Solve(dircol)
        print(result)
        # print("Timestep = {}".format(dircol.fixed_timestep()))

        u_trajectory = dircol.ReconstructInputTrajectory(result)
        print(u_trajectory)
        for t in range(0, int(u_trajectory.end_time()*100), 5):
            t /= 100
            print("t = {}, v = {}".format(t, u_trajectory.value(t)))

        # times = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 5)
        # u_lookup = np.vectorize(u_trajectory.value)
        # u_values = u_lookup(times)

        # print("Result success? {}, u = {}".format(result.is_success(), u_values))


# tmp = DrakeTrajectoryOptimization()