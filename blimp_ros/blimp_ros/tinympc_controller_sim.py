########################################################
# tinympc_controller_sim.py
#
# NOT USED FOR ANY BLIMP ACTIVITIES,
# simple script that simulates the tinympc controller
# to ensure that it was a feasible approach
#
########################################################





import tinympc

import numpy as np
from scipy.integrate import solve_ivp
from scipy.signal import cont2discrete
import matplotlib.pyplot as plt


DRONE_PROTOTYPE_2_MASS = 46/1000 #kg
DRONE_PROTOTYPE_0_MASS = 63/1000 #kg
BALLOON_MASS = .0545 #kg
m = DRONE_PROTOTYPE_2_MASS + BALLOON_MASS

fb = 9.804 #m/s^2
G = 9.81 #m/s^2
VOLTAGE_CONSTANT = .00714
U_ERROR = 3.0
MAX_VOLTAGE = 0.5
Dz = 0#0.0480
Dwy = 0#0.000980
b = .000980

Fb = m*(G-fb)

H_ENV = 40/100 #m
H_GON = 4/100 #m
D_ENV = 81.28/100 #m

D_MT = H_ENV/2 + H_GON #distance between CM and gondola
D_VM = 0.0971 #m, found from paper, distance between CM and Center of lift

Icm = 0.005821 #Icm, should be roughly correct

# A = np.array([[0,1],[-Fb/Icm*D_VM, -b/Icm]])
# B = np.array([[0],[D_MT/Icm]])

MAX_THRUST = 0.3**2*VOLTAGE_CONSTANT
# MAX_THRUST = 0.3

A = np.array([
    [0,0,1,0],
    [0,0,0,1],
    [0,0,0,0],
    [0,-Fb/Icm*D_VM,0,-b/Icm]
],dtype=np.float64)

B = np.array([
    [0 , 0],
    [0 , 0],
    [1/m , 0],
    [0 , D_MT/Icm]
],dtype=np.float64)

# B = B[:,0:1] + B[:,1:2]

pitch_dynamics = lambda X,u: 1/Icm * (-b*X[3] - Fb*D_VM*np.sin(X[1])+D_MT*u) 
# pitch_dynamics = lambda X,u: 1/Icm * (-b*X[1] - Fb*D_VM*np.sin(X[0])+D_MT*u) 
forward_dynamics = lambda X,u: u/m

X0 = [0.05,0.0,0.01,0.0]
Q = np.diag([0.0,0.0,10.0,0.0])
R = np.diag([5.0,0.0])
goal = [1.0,0.0,0.5,0.0]#x,theta,vx,wy
u0 = np.array([0.0,Fb*D_VM/D_MT*np.sin(goal[0])])
N=20
solver = tinympc.TinyMPC()

dt = 0.02 # 50Hz
Ad, Bd, _, _, _ = cont2discrete((A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], B.shape[1]))), dt, method='zoh')

solver.setup(Ad,Bd,Q,R,N,rho=1.0,verbose=False)
solver.set_x0(np.array(X0))
solver.set_x_ref(np.array(goal))
solver.set_u_ref(np.array(u0))
solver.set_bound_constraints(np.array([-float('inf')]*4),np.array([float('inf')]*4),np.array([-MAX_VOLTAGE,-MAX_VOLTAGE]),np.array([MAX_VOLTAGE,MAX_VOLTAGE]))

dirn = lambda x: x/abs(x)
def model(t,X):
    solver.set_x0(X)
    u = solver.solve()['controls']
    u = u[0]+u[1]
    # print(u[0],u[1])
    # if dirn(u[0]) == dirn(u[1]):
    #     u = u[0]
    # elif dirn(u[0]) != dirn(u[1]):
    #     # u = min(u[0],u[1])
    #     if X[1] > 0.3:
    #         u = u[1]
    #     else:
    #         u = u[0]
    #     # u = (u[0]+u[1])/2
    # if abs(u) > MAX_THRUST:
    #     u = u/abs(u)*MAX_THRUST
    # u = u[0]+u
    if abs(u) > MAX_THRUST:
        u = u/abs(u)*MAX_THRUST    
    # if abs(u[1]) > MAX_THRUST:
    #     u[1] = u[1]/abs(u[1])*MAX_THRUST
    x_acc = u/m
    pitch_acc = pitch_dynamics(X,u)
    
    return [X[2],X[3],x_acc,pitch_acc]


states = solve_ivp(model,[0,50],np.array(X0))
plt.plot(states.t,states.y[2,:])
plt.plot(states.t,1*np.ones_like(states.t))
plt.title("X Position over time")
plt.show()

plt.plot(states.t,states.y[1,:])
plt.plot(states.t,0.0*np.ones_like(states.t))
# plt.ylim([-1,1])
plt.title("Pitch over time")
plt.show()