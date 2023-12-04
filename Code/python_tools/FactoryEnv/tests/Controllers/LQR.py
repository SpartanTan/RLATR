import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.linalg import inv
import cvxpy as cp

def angle_wrap(ang):
    """
    Wraps the angle to the range [-pi, pi).
    """
    return (ang + np.pi) % (2 * np.pi) - np.pi

def compute_angle_error(theta_current, theta_ref):
    """
    Computes the smallest difference between two angles.
    """
    return angle_wrap(theta_current - theta_ref)


def lqr_control(x_k, x_ref, R, L, Delta_t, Q, R_input, u):
    wr, wl = u
    theta_k = x_k[2]
    # Update B_lin for the current heading angle theta_k
    B_lin = np.array([[R / 2 * np.cos(theta_k) * Delta_t, R / 2 * np.cos(theta_k) * Delta_t],
                      [R / 2 * np.sin(theta_k) * Delta_t, R / 2 * np.sin(theta_k) * Delta_t],
                      [(R / L) * Delta_t, -(R / L) * Delta_t]])

    # Linearized A matrix (remains constant if the system is time-invariant)
    A_lin = np.array([[1, 0, -R/2*Delta_t*(wr+wl)*np.sin(theta_k)],
                      [0, 1, R/2*Delta_t*(wr+wl)*np.cos(theta_k)],
                      [0, 0, 1]])
    # A_lin = np.array([[1, 0, 0],
    #                   [0, 1, 0],
    #                   [0, 0, 1]])
    # Q_modified = Q.copy()
    # Q_modified[2,2] = 0
    # Solve the discrete-time algebraic Riccati equation to find P
    P = solve_discrete_are(A_lin, B_lin, Q, R_input)

    # Compute the LQR gain matrix K
    K = -inv(R_input + B_lin.T @ P @ B_lin) @ B_lin.T @ P @ A_lin
    # K = inv(R_input) @ (B_lin.T @ P)
    # Compute the control input u
    theta_error = compute_angle_error(theta_k, x_ref[2])
    theta_error = 0.0 # ignore the heading error
    state_error = np.array([x_k[0] - x_ref[0], x_k[1] - x_ref[1], theta_error])
    u = K @ (state_error)

    return u

def mpc_control(x_k, x_ref, R, L, Delta_t, Q, Qf, R_input, u_min, u_max, N, u):
    wr, wl = u
    theta_k = x_k[2]
    theta_ref = x_ref[2]
    # Linearized A matrix (remains constant if the system is time-invariant)
    # A_lin = np.array([[1, 0, -R/2*Delta_t*(wr+wl)*np.sin(theta_k)],
    #                   [0, 1, R/2*Delta_t*(wr+wl)*np.cos(theta_k)],
    #                   [0, 0, 1]])
    A_lin = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
    # Update B_lin for the current heading angle theta_k
    B_lin = np.array([[R / 2 * np.cos(theta_k) * Delta_t, R / 2 * np.cos(theta_k) * Delta_t],
                      [R / 2 * np.sin(theta_k) * Delta_t, R / 2 * np.sin(theta_k) * Delta_t],
                      [(R / L) * Delta_t, -(R / L) * Delta_t]])
    
    # State (x) and input (u) variables for the optimization problem
    x = cp.Variable((3, N+1))
    u = cp.Variable((2, N))

    # Cost function
    cost = 0
    constraints = []

    # Populate the cost and constraints in a loop
    for k in range(N):
        cost += cp.quad_form(x[:, k] - x_ref, Q) + cp.quad_form(u[:, k], R_input)
        constraints += [
            x[:, k+1] == A_lin @ x[:, k] + B_lin @ u[:, k],
            u[:, k] >= u_min,
            u[:, k] <= u_max
        ]
    # Create a vector for the reference that ignores theta
    cost += cp.quad_form(x[:, N] - x_ref, Q)

    # # Ignore theta in the reference
    # for k in range(N):
    #     # Create the state error, setting theta error to zero
    #     state_error_no_theta = cp.vstack([x[0, k] - x_ref[0], 
    #                                       x[1, k] - x_ref[1], 
    #                                       0])  # Set theta error to zero

    #     cost += cp.quad_form(state_error_no_theta, Q) + cp.quad_form(u[:, k], R_input)
    #     constraints += [
    #         x[:, k+1] == A_lin @ x[:, k] + B_lin @ u[:, k],
    #         u[:, k] >= u_min,
    #         u[:, k] <= u_max
    #     ]
    # final_state_error_no_theta = cp.vstack([x[0, N] - x_ref[0], 
    #                                         x[1, N] - x_ref[1], 
    #                                         0])  # Set theta error to zero
    # cost += cp.quad_form(final_state_error_no_theta, Q)

    # Improved heading error
    # for k in range(N):
    #     # Position error
    #     position_error = cp.reshape(x[:2, k] - x_ref[:2], (2, 1))  # 2x1 vector

    #     # Angle wrapping constraint
    #     theta_error = cp.Variable(1)
    #     constraints += [
    #         cp.abs(theta_error) <= np.pi,
    #         theta_error == x[2, k] - theta_ref,  # Adjust for angle wrapping
    #     ]

    #     # Ensure theta_error is a 2D column vector
    #     theta_error_reshaped = cp.reshape(theta_error, (1, 1))

    #     # Stack the position error and the reshaped theta error
    #     state_error = cp.vstack([position_error, theta_error_reshaped])

    #     # Cost function update
    #     cost += cp.quad_form(state_error, Q) + cp.quad_form(u[:, k], R_input)
    #     constraints += [
    #         x[:, k+1] == A_lin @ x[:, k] + B_lin @ u[:, k],
    #         u[:, k] >= u_min,
    #         u[:, k] <= u_max
    #     ]

    # # Final state error handling...
    # final_position_error = cp.reshape(x[:2, N] - x_ref[:2], (2, 1))

    # final_theta_error = cp.Variable(1)
    # constraints += [
    #     cp.abs(final_theta_error) <= np.pi,
    #     final_theta_error == x[2, N] - theta_ref,
    # ]

    # final_theta_error_reshaped = cp.reshape(final_theta_error, (1, 1))
    # final_state_error = cp.vstack([final_position_error, final_theta_error_reshaped])

    # cost += cp.quad_form(final_state_error, Qf)

    # Initial condition constraint
    constraints += [x[:, 0] == x_k]

    # Set up and solve the optimization problem
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()

    # Check if the problem was solved
    if prob.status in ["infeasible", "unbounded"]:
        raise ValueError("The optimization problem is infeasible or unbounded")

    # Extract the optimal input
    u_optimal = u[:, 0].value
    # Extract the state predictions
    x_optimal = x.value
    return u_optimal, x_optimal

if __name__ == "__main__":
    # Define the system parameters
    R = 0.125 # radius of the wheels
    L = 0.48 # distance between the wheels
    Delta_t = 0.1 # sampling time

    # Define the weighting matrices Q and R
    Q = np.diag([1, 1, 0.1])  # State weighting matrix
    R_input = np.diag([0.1, 0.1])  # Control input weighting matrix

    u_min = np.array([-0.8, -0.8])    # Minimum control input
    u_max = np.array([0.8, 0.8])    # Maximum control input
    N = 20

    # Example usage:
    theta_k = 0.0 # Current heading angle
    x_k = np.array([0, 0, 0.0])  # Current state [x, y, theta]
    x_ref = np.array([0, 1, np.pi/2])  # Reference state [x_ref, y_ref, theta_ref]
    u = [0,0]
    # Get the control input
    u = lqr_control(theta_k, x_k, x_ref, R, L, Delta_t, Q, R_input, u)
    print("Control input u:", u)

    # Get the control input using MPC
    u = mpc_control(theta_k, x_k, x_ref, R, L, Delta_t, Q, R_input, u_min, u_max, N, u)
    print("Optimal control input u:", u)