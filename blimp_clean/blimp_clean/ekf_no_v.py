import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_discrete_are
import matplotlib.pyplot as plt


RHO_AIR = 1.225 #kg/m^3
RHO_HE = 0.165 # kg/m^3
M_chassis = 46/1000 #kg
M_AZ = 0.0545/2 #kg
g = 9.81

# [Fb, V, Kv, Cd]
TRUE_PARAMS = [9.68, M_chassis / (RHO_AIR - 2*RHO_HE) + 0.2, 0.0714, 0.0380]

us = []
def eom(t, X, u):
    Fb0 = TRUE_PARAMS[0]
    V0  = TRUE_PARAMS[1]
    Kv0 = TRUE_PARAMS[2]
    Dz0 = TRUE_PARAMS[3]
    m    = M_chassis + M_AZ + RHO_HE*V0
    m_rb = m - M_AZ
    d0   = 0.0
    dX = np.zeros_like(X)
    dX[0] = X[1]
    dX[1] = (1.0/m) * ((RHO_AIR - RHO_HE)*g*V0
                       - m_rb*g
                       + Kv0*(u - d0)
                       - Dz0*X[1])
    return dX
def gen_measurements(dt, plot=False, T=5.0, sigma_proc=0.05,
                     sigma_pos=0.05, sigma_vel=0.05, seed=None):
    global us
    rng = np.random.default_rng(seed)
    t_eval = np.arange(0.0, T + dt/2, dt)
    nt = len(t_eval)
    X = np.zeros(2)
    xs = np.zeros((2, nt))
    us = []
    xs[:, 0] = X
    V0 = TRUE_PARAMS[1]
    Kv0 = TRUE_PARAMS[2]
    Dz0 = TRUE_PARAMS[3]
    m = M_chassis + M_AZ + RHO_HE*V0
    m_rb = m - M_AZ
    d0 = 0.0
    for k in range(nt - 1):
        control_state = np.array([X[0], X[1], V0, Kv0, Dz0])
        u_k = controller.get_control(control_state, dt, m, m_rb, d0).item()
        us.append(np.array([[u_k]]))
        sol = solve_ivp(eom,
                        (t_eval[k], t_eval[k+1]),
                        X,
                        args=(u_k,),
                        rtol=1e-8, atol=1e-10)
        X = sol.y[:, -1].copy()
        X[1] += sigma_proc * rng.standard_normal()
        xs[:, k+1] = X
    v = sigma_pos * rng.standard_normal((1, nt))
    z = xs[0:1, :] + v
    true = [xs]
    if plot:
        fig, ax = plt.subplots(2, 1, sharex=True)
        ax[0].plot(t_eval, xs[0], label='true z')
        ax[0].scatter(t_eval, z[0], s=8, label='meas z')
        ax[0].set_ylabel('z [m]'); ax[0].legend()
        ax[1].plot(t_eval, xs[1], label='true zdot')
        ax[1].set_xlabel('t [s]'); ax[1].set_ylabel('zdot [m/s]'); ax[1].legend()
        plt.show()
    return z, true

def get_FG(X, d, dt):
    z, zdot, V, Kv, Cd = X[:,0]
    d0 = 0.0
    m = M_chassis + M_AZ + RHO_HE*V
    m_RB = M_chassis + RHO_HE*V
    # dV = -(RHO_AIR - RHO_HE)*g*(RHO_HE + V * (m_RB))**(-2)*(m_RB)
    u = d[0,0]
    N = (RHO_AIR - RHO_HE)*g*V - m_RB*g + Kv*(u - d0) - Cd*zdot
    dN_dV = (RHO_AIR - 2*RHO_HE)*g
    dm_dV = RHO_HE
    dV = (dN_dV*m - N*dm_dV) / m**2
    F = np.array([
        [1, dt, 0, 0, 0],
        [0, 1 - 1/m * Cd * dt, dV*dt, 1/m*(u-d0)*dt, -zdot/m*dt],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])

    G = np.array([
        [0],
        [1/m*Kv*dt],
        [0],
        [0],
        [0]
    ])

    return F, G

def ekf(z, u, dt):

    nz = z.shape[1]
    
    H = np.array([[1,0,0,0,0]])
    R = np.array([[0.01**2]])

    Q = np.eye(1)*0.5**2
    Xk = np.array([0.0,0.0,0.04,0.00714,0.0480]).reshape((5,1))
    P = np.diag([0.1**2, 0.5**2, 2.0**2, 10.0**2, 1.0**2])
    xs = []
    ps = []
    for k in range(nz - 1):
        x, xdot, V, Kv, Cd = Xk
        m = M_chassis + RHO_HE*V + M_AZ
        m_RB = M_chassis + RHO_HE*V

        # Prediction Step
        Xnext = Xk.copy()
        Xnext[0] = Xk[1] # velocity
        Xnext[1] =  1/m * ( (RHO_AIR-RHO_HE)*g*V - m_RB*g + Kv*u[k] - Cd*Xk[1] ) # acceleration
        Xnext[2] = 0 # parameters are constant
        Xnext[3] = 0
        Xnext[4] = 0

        Xp = Xk + dt*Xnext
        # Predict covariance
        F, G = get_FG(Xp, u[k], dt)
        P = F@P@F.T + G@Q@G.T

        # Kalman Gain
        S = H@P@H.T + R
        K = P@H.T@np.linalg.inv(S)

        # Innovation
        nu = z[:,k+1] - H@Xp

        #Update
        Xk = Xp + K @ nu

        P = (np.eye(5) - K@H)@P@(np.eye(5)-K@H).T + K@R@K.T
        xs.append(Xk)
        ps.append(P)

    return xs,ps

class LQR(object):
    def __init__(self):
        self.Q = np.diag([10.0,5.0])
        self.R = np.array([[5.0]])
        return
    
    def get_AB(self, X, dt, m):
        z, zdot, V, Kv, Cd = X

        A = np.array([
            [1, dt],
            [0, 1 - Cd/m*dt]
        ])
        B = np.array([
            [0],
            [dt/m]
        ])

        return A,B

    def get_control(self, X, dt, m, m_RB, d0):
        z, zdot, V, Kv, Cd = X
        Xc = np.array(X[:2]).reshape((2,1))

        goal = np.array([[1.5],[0.0]])

        Fb = (RHO_AIR-RHO_HE)*g*V
        A, B = self.get_AB(X, dt, m)
        u0 = m_RB*g - Fb

        P = solve_discrete_are(A,B,self.Q,self.R)
        K = np.linalg.inv(B.T @ P @ B + self.R) @ (B.T @ P @ A)
        thrust = u0 - K@(Xc - goal)
        d = d0 + thrust/Kv
        
        return d

controller = LQR()
dt = 0.01
z, true_vals = gen_measurements(dt,False,sigma_pos=0.01, sigma_proc=0.01, sigma_vel=0.1)
xhat, phat = ekf(z, us, dt)

true = true_vals[0]
xhat = np.array(xhat)[:, :, 0].T
phat = np.array(phat)

t_true = np.arange(true.shape[1]) * dt
t_hat = t_true[1:]

V0 = M_chassis / (RHO_AIR - 2*RHO_HE)
true_states = np.vstack([
    true,
    np.full(true.shape[1], TRUE_PARAMS[1]),
    np.full(true.shape[1], TRUE_PARAMS[2]),
    np.full(true.shape[1], TRUE_PARAMS[3]),
])

state_labels = ["z [m]", "zdot [m/s]", "V [m^3]", "Kv", "Cd"]
fig, axes = plt.subplots(5, 1, sharex=True, figsize=(10, 9))

for i, ax in enumerate(axes):
    sigma = np.sqrt(np.maximum(phat[:, i, i], 0.0))
    ax.plot(t_true, true_states[i], label="true", color="black")
    ax.plot(t_hat, xhat[i], label="estimate", color="tab:blue")
    ax.fill_between(
        t_hat,
        xhat[i] - 2*sigma,
        xhat[i] + 2*sigma,
        color="tab:blue",
        alpha=0.2,
        label="±2σ" if i == 0 else None,
    )
    ax.set_ylabel(state_labels[i])
    ax.grid(True)

axes[0].legend()
axes[-1].set_xlabel("time [s]")
fig.tight_layout()

param_labels = state_labels[2:]
param_errors = xhat[2:5] - true_states[2:5, 1:]

fig_err, err_axes = plt.subplots(3, 1, sharex=True, figsize=(10, 6))
for i, ax in enumerate(err_axes):
    ax.plot(t_hat, param_errors[i], color="tab:red")
    ax.axhline(0.0, color="black", linewidth=1)
    ax.set_ylabel(f"{param_labels[i]} error")
    ax.grid(True)

err_axes[-1].set_xlabel("time [s]")
fig_err.tight_layout()
plt.show()
