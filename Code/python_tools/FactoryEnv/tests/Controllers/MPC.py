import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# Parameters
R = 0.05  # Radius of the wheels
L = 0.15  # Distance between the wheels
dt = 0.1  # Time step
N = 20    # Prediction horizon

# State and input dimensions
nx = 3
nu = 2

# Define the symbolic states and controls as SX variables
x, y, theta = ca.SX.sym('x'), ca.SX.sym('y'), ca.SX.sym('theta')
states = ca.vertcat(x, y, theta)
omega_R, omega_L = ca.SX.sym('omega_R'), ca.SX.sym('omega_L')
controls = ca.vertcat(omega_R, omega_L)

# State equation
rhs = ca.vertcat(
    R/2 * (omega_R + omega_L) * ca.cos(theta),
    R/2 * (omega_R + omega_L) * ca.sin(theta),
    R/L * (omega_R - omega_L)
)
f = ca.Function('f', [states, controls], [rhs])

# MPC setup
U = ca.SX.sym('U', nu, N)
P = ca.SX.sym('P', nx + nx)  # Parameter vector (initial state + reference state)
X = ca.SX.sym('X', nx, N+1)  # Decision variable for state trajectory

# Objective function and constraints
obj = 0  # Objective function
Q = ca.diag([1, 1, 0.1])  # State cost
R_mat = ca.diag([0.1, 0.1])  # Control cost

for k in range(N):
    st = X[:, k]  # Current state
    cont = U[:, k]  # Current control
    # State cost
    obj += ca.mtimes([(st - P[nx:]).T, Q, (st - P[nx:])])
    # Control cost
    obj += ca.mtimes([cont.T, R_mat, cont])
    # Dynamics constraint
    st_next = X[:, k+1]
    f_value = f(st, cont) * dt
    obj += ca.mtimes([st_next - (st + f_value)].T, st_next - (st + f_value))

# Optimization problem
nlp_prob = {'f': obj, 'x': ca.vertcat(U.reshape((-1, 1)), X.reshape((-1, 1))),
            'g': ca.vertcat(*g), 'p': P}

# Solver options
opts = {'ipopt.print_level': 0, 'print_time': 0}
solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

# Simulation
x0 = [0, 0, 0]       # Initial condition
xs = [1, 2, 0]       # Final state
xx = np.zeros((nx, N+1))
uu = np.zeros((nu, N))
xx[:, 0] = x0

# MPC loop
for i in range(N):
    p = ca.vertcat(*x0, *xs)
    sol = solver(x0=ca.vertcat(U, X).reshape((-1, 1)), p=p, lbg=0, ubg=0)
    u = ca.reshape(sol['x'][0:N*nu], nu, N).full()[:, 0]
    x0 = ca.DM(f(x0, u)).full().flatten()
    uu[:, i] = u
    xx[:, i+1] = x0

# Plotting
plt.figure()
plt.plot(xx[0, :], xx[1, :], label='Robot Path')
plt.plot(xs[0], xs[1], 'ro', label='Target')
plt.xlabel('X position')
plt.ylabel('Y position')
plt.title('MPC Controlled Path of Differential Drive Robot')
plt.legend()
plt.grid()
plt.show()
