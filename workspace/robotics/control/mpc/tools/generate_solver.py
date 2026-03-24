#!/usr/bin/env python3
"""
Generate an acados NMPC solver for the unicycle kinematic model.

Prerequisites:
    pip install casadi acados-template

Usage:
    python generate_solver.py

Output:
    Generated C code in ../src/generated/

This script defines an Optimal Control Problem (OCP) for a unicycle robot
using CASAdi for symbolic dynamics and acados for solver generation.

The generated solver can be called from C++ via the acados C API.
"""
import os
import sys
import numpy as np

# Attempt to import CASAdi and acados
try:
    from casadi import SX, vertcat, sin, cos, tan, Function
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
except ImportError as e:
    print(f"Error: {e}")
    print("Install dependencies: pip install casadi acados-template")
    print("Also ensure acados is installed: https://docs.acados.org/installation/")
    sys.exit(1)


def create_unicycle_model():
    """
    Create a CASAdi symbolic model of the unicycle (differential-drive) robot.

    State:  x = [px, py, theta]  (position x, position y, heading angle)
    Control: u = [v, omega]       (linear velocity, angular velocity)

    Continuous dynamics:
        px_dot    = v * cos(theta)
        py_dot    = v * sin(theta)
        theta_dot = omega
    """
    model = AcadosModel()
    model.name = "unicycle"

    # State variables
    px = SX.sym("px")
    py = SX.sym("py")
    theta = SX.sym("theta")
    x = vertcat(px, py, theta)

    # Control variables
    v = SX.sym("v")
    omega = SX.sym("omega")
    u = vertcat(v, omega)

    # State derivatives (continuous-time dynamics)
    px_dot = v * cos(theta)
    py_dot = v * sin(theta)
    theta_dot = omega

    # Explicit ODE: xdot = f(x, u)
    f_expl = vertcat(px_dot, py_dot, theta_dot)

    # Implicit ODE: 0 = f_impl(x, xdot, u) = xdot - f(x, u)
    xdot = SX.sym("xdot", 3)

    model.x = x
    model.u = u
    model.xdot = xdot
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    return model


def create_ocp(model, N=20, T=2.0):
    """
    Create the Optimal Control Problem (OCP) for trajectory tracking.

    Cost: sum_{k=0}^{N-1} [ (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k ]
          + (x_N - x_ref_N)^T Qf (x_N - x_ref_N)

    Constraints:
        - Dynamics: unicycle model (exact, not linearized)
        - Control bounds: |v| <= v_max, |omega| <= omega_max
        - State bounds: none (can be added)

    Parameters:
        N: prediction horizon (number of steps)
        T: prediction time (seconds)
    """
    ocp = AcadosOcp()
    ocp.model = model

    # Dimensions
    nx = model.x.shape[0]  # 3 (px, py, theta)
    nu = model.u.shape[0]  # 2 (v, omega)

    ocp.dims.N = N

    # Cost: NONLINEAR_LS type
    # y = [x; u] for stages 0..N-1
    # y_e = [x] for terminal stage N
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x

    # Weight matrices
    Q = np.diag([10.0, 10.0, 1.0])       # state tracking weights [px, py, theta]
    R = np.diag([0.1, 0.1])              # control effort weights [v, omega]
    Qf = np.diag([100.0, 100.0, 10.0])   # terminal state weights

    ocp.cost.W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    ocp.cost.W_e = Qf

    # Reference (updated at runtime)
    ocp.cost.yref = np.zeros(nx + nu)     # [x_ref; u_ref]
    ocp.cost.yref_e = np.zeros(nx)        # [x_ref_terminal]

    # Control constraints
    v_max = 1.0       # m/s
    omega_max = 2.0   # rad/s

    ocp.constraints.lbu = np.array([-v_max, -omega_max])
    ocp.constraints.ubu = np.array([v_max, omega_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # Initial state constraint (updated at runtime)
    ocp.constraints.x0 = np.zeros(nx)

    # Solver options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # Real-Time Iteration
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"  # Explicit Runge-Kutta
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.tf = T

    return ocp


def generate_solver():
    """Generate the acados solver C code."""
    model = create_unicycle_model()
    ocp = create_ocp(model)

    # Output directory for generated code
    output_dir = os.path.join(os.path.dirname(__file__), "..", "src", "generated")
    os.makedirs(output_dir, exist_ok=True)

    # Generate solver
    solver = AcadosOcpSolver(
        ocp, json_file=os.path.join(output_dir, "acados_ocp.json")
    )

    print(f"Solver generated successfully in {output_dir}")
    print(f"  Model: {model.name}")
    print(f"  Horizon: N={ocp.dims.N}, T={ocp.solver_options.tf}s")
    print(f"  QP solver: {ocp.solver_options.qp_solver}")
    print(f"  NLP solver: {ocp.solver_options.nlp_solver_type}")
    print()
    print("To use in C++, rebuild the project with acados installed:")
    print("  cmake -B build -S workspace -DCMAKE_BUILD_TYPE=Release")
    print("  cmake --build build -j$(nproc)")

    return solver


if __name__ == "__main__":
    generate_solver()
