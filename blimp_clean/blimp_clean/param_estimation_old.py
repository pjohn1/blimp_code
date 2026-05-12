import numpy as np
from rclpy.node import Node
from blimp_msgs.msg import MotorMsg, OptiTrackPose
from std_msgs.msg import Bool, Float32MultiArray

from scipy.linalg import solve_discrete_are
from scipy.signal import place_poles
import os
import threading

import tinympc
import time
RHO_AIR = 1.225 #kg/m^3
RHO_HE = 0.165 # kg/m^3
M_chassis = 46/1000 #kg
M_AZ = 0.0545/2 #kg
g = 9.81

MAX_VOLTAGE = 0.8

class Model(object):
    def __init__(self, d0=0.0):
        '''
        Initialize model update class

        '''
        self.d0 = d0
        V0 = 0.0514
        self.m = M_chassis + RHO_HE*V0 + M_AZ
        self.m_RB = self.m - M_AZ
    
    def get_FG(self,X, d, dt, d0):
        
        z, zdot, V, Kv, Cd = X
        self.m = M_chassis + RHO_HE*V + M_AZ
        self.m_RB = self.m - M_AZ

        u = d

        # Derivative w.r.t volume (parameter is on LHS and RHS, quotient rule)
        N = (RHO_AIR - RHO_HE)*g*V - self.m_RB*g + 2*Kv*(u - d0) - Cd*zdot # RHS
        dN_dV = (RHO_AIR - 2*RHO_HE)*g #Derviative wrt RHS
        dm_dV = RHO_HE # Derivative w.r.t mass
        dV = (dN_dV*self.m - N*dm_dV) / self.m**2 #Quotient rule

        # Make F and G
        F = np.array([
            [1, dt, 0, 0, 0],
            [0, 1 - 1/self.m * Cd * dt, dV*dt, 1/self.m*(u-d0)*dt, -zdot/self.m*dt],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        G = np.array([
            [0],
            [1/self.m*Kv*dt],
            [0],
            [0],
            [0]
        ])

        return F, G

class ParamEstimation(Node):
    def __init__(self, ns, com_port):
        super().__init__('param_estimation',namespace=ns)
        self.ns = ns
        self.com = com_port

        # ROS Setup
        self.create_subscription(OptiTrackPose,f'optitrack_node/pose',self.ekf,5)
        self.create_subscription(Bool, f'start_calibration', self.start, 2)
        self.motor_pub = self.create_publisher(MotorMsg,f'motor_cmd',5) #Goal publisher to control testing mode
        self.covar_pub = self.create_publisher(Float32MultiArray, f'covariance',5) # These update the graphs in the GUI
        self.pred_pub = self.create_publisher(Float32MultiArray, f'state_est',5)
        
        # Initializers
        self.run_calibration = False
        self.finished_calibration = False
        self.measure_v = True


        # Initial values and model
        V0 = 0.0514 
        Kv0 = 0.714
        Dz0 = 0.0480
        # V0 = 0.010816676121561977
        # Kv0 = 0.4589
        # Dz0 = 0.6227
        self.d0 = 0.0
        self.N_avg = 1
        self.N_ts = 10

        self.X = [0.0,0.0,V0, Kv0, Dz0] # Tracks the current position and parameters
        self.model = Model()

        #Initialize controller
        # self.controller = LQR()
        self.controller = MPC(5,self)

        # Tracking
        self.last_time = None
        self.last_z = None
        self.zs = []
        self.times = []
        self.update_lock = threading.Lock()

        # Plotting
        self.state_estimates = []
        self.covariance_estimates = []
        self.test_statistics = []
        self.avg_test_statistics = []

        self.last_P = None

        self.init_params()
    
    def init_params(self):
        if self.measure_v:
            self.H = np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]
            ])

            self.R = np.diag([0.01**2,0.1**2]) # Uncertainty in measurements
        else:
            self.H = np.array([[1,0,0,0,0]])
            self.R = np.array([[0.01**2]])

        self.Q = np.eye(1)*0.5**2 # Uncertainty in model
        self.P = P = np.diag([0.1**2, 1.0**2, 1.0**2, 5.0**2, 1.0**2]) # Uncertainty in estimate



    def start(self,msg):
        self.run_calibration = msg.data

    def ekf(self, msg):
        if not self.finished_calibration:
            if len(self.zs)==self.N_avg:
                dt = msg.time - self.times[0] 
                v = (msg.z - self.zs[0]) / dt
                if abs(v) > 1.5:
                    return

                if self.measure_v:
                    measurement = np.array([msg.z,v])
                else:
                    measurement = msg.z
                self.X[0] = msg.z
                self.X[1] = v

                self.zs = []
                self.times = []


                if self.run_calibration:
                    with self.update_lock:
                        t0 = time.time()
                        # Get voltage output
                        u = self.controller.get_control(self.X,np.array([1.5,0.0]))
                        run_ekf = True
                        if run_ekf:

                            Xk = np.array(self.X.copy())
                            x, xdot, V, Kv, Cd = Xk

                            # Predict
                            Xnext = Xk.copy()
                            Xnext[0] = Xk[1] # velocity
                            Xnext[1] =  1/self.model.m * ( (RHO_AIR-RHO_HE)*g*V - self.model.m_RB*g + 2*Kv*u - Cd*Xk[1] ) # acceleration
                            Xnext[2] = 0 # parameters are constant
                            Xnext[3] = 0
                            Xnext[4] = 0

                            Xp = Xk + dt*Xnext # Predicted state
                            F, G = self.model.get_FG(Xp, u, dt,self.d0)
                            Pp = F@self.P@F.T + G@self.Q@G.T #Predicted covariance

                            # Kalman gain
                            S = self.H@Pp@self.H.T + self.R
                            K = Pp@self.H.T@np.linalg.inv(S)

                            # Innovation
                            nu = measurement - self.H@Xp
                            self.get_logger().info(f'Shapes: {nu.shape, S.shape}\n\n\n')
                            test_statistic = nu.T @ np.linalg.inv(S) @ nu

                            #Update
                            self.X = Xp + K @ nu
                            self.P = (np.eye(5) - K@self.H)@Pp@(np.eye(5)-K@self.H).T + K@self.R@K.T

                            self.X[2] = max(1e-6, self.X[2])
                            self.X[3] = max(1e-6, self.X[3])
                            self.X[4] = max(1e-6, self.X[4])

                            self.state_estimates.append(self.X)
                            self.covariance_estimates.append(self.P)
                            self.test_statistics.append(test_statistic)

                            if len(self.test_statistics) > self.N_ts:
                                self.avg_test_statistics.append(np.mean(self.test_statistics[-self.N_ts:]))

                            if self.last_P is not None and np.linalg.norm(self.P[2:, 2:] - self.last_P[2:,2:]) < 1e-6:
                                self.finished_calibration = True
                                self.finished_time = msg.time
                                self.get_logger().info(f'Estimated V: {self.X[2]}, Kv: {self.X[3]}, Cd: {self.X[4]}')
                                
                                for i in range(10):
                                    cmd = MotorMsg()
                                    cmd.id = msg.id
                                    cmd.com = self.com
                                    cmd.voltages = Float32MultiArray(data=list(np.array([0.0,0.0,0.0,0.0,0.0,0.0])))
                                    self.motor_pub.publish(cmd) 
                                return
                            
                            self.last_P = self.P
                            # Publish motor command and visualization messages
                            self.covar_pub.publish(msg=Float32MultiArray(data=list(self.P.flatten())))
                            self.pred_pub.publish(msg=Float32MultiArray(data=list(self.X.flatten())))

                            cmd = MotorMsg()
                            cmd.id = msg.id
                            cmd.com = self.com
                            cmd.voltages = Float32MultiArray(data=list(np.array([0.0,0.0,u,-u,0.0,0.0])))
                            self.motor_pub.publish(cmd) 
                            self.get_logger().info(f'Currently running at {round(1/(time.time()-t0),1)}Hz')
            else:
                self.zs.append(msg.z)
                self.times.append(msg.time)
        
        else:
            os.makedirs(self.ns, exist_ok=True)
            np.save(self.ns + '/state_estimates.npy', self.state_estimates)
            np.save(self.ns + '/covariance_estimates.npy', self.covariance_estimates)
            np.save(self.ns + '/finished_time.npy', self.finished_time)
            np.save(self.ns + '/test_statistics.npy', self.test_statistics)
            np.save(self.ns + '/avg_test_statistics.npy', self.avg_test_statistics)


class MPC(object):
    '''
    MPC Control using tinyMPC
    Pros: Fast, Very effective
    Cons: Not realistic for decentralized system with cheap hardware
    '''
    def __init__(self,N,node):

        self.node = node
        #Discretize continuous A,B matrices
        dt = 1/(100/node.N_avg)
        # Ad, Bd, _, _, _ = \
        #             cont2discrete((A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], B.shape[1]))), dt, method='zoh')
        V = 0.0514 
        self.Kv = 0.714
        Cd = 0.0480
        m = M_chassis + RHO_HE*V + M_AZ
        m_RB = m - M_AZ

        A = np.array([
            [1, dt],
            [0, 1 - Cd/m*dt]
        ])
        B = np.array([
            [0],
            [dt/m]
        ])

        Fb = (RHO_AIR-RHO_HE)*g*V
        u0 = m_RB*g - Fb

        Q_alt = np.diag([10.0,5.0])
        R_alt = np.array([[3.0]])

        self.Ad = A
        self.Bd = B
        self.Q = np.asarray(Q_alt, dtype=np.float64)
        self.R = np.asarray(R_alt, dtype=np.float64)
        self.N = N
        self.rho = 1.0

        self.solver = tinympc.TinyMPC()
        # Protects TinyMPC C++ state from concurrent solve/setup under MultiThreadedExecutor
        self.solver_lock = threading.Lock()
        # self.solver.set_x_ref([self.node.x_goal[0],self.node.pitch_goal[0],self.node.x_goal[1],self.pitch_goal[1]])
        self.u0 = [u0]
        self.solver.setup(self.Ad,self.Bd,self.Q,self.R,self.N,rho=self.rho,verbose=False)
        self.solver.set_u_ref(np.array(self.u0))


        self.thrust_to_voltage = lambda thrust: thrust/abs(thrust) * np.sqrt(abs(thrust))/VOLTAGE_CONSTANT/100

    def update_u0(self,new_u0):
        self.u0 = new_u0

    def get_control(self,pose,goal):

        state = np.array(pose[:2])
        local_goal = goal.copy() # dont change goal array

        with self.solver_lock:
            self.solver.set_x0(state)
            self.solver.set_u_ref(np.array(self.u0))
            self.solver.set_x_ref(local_goal)
            thrust = self.solver.solve()['controls'][0]

        u = thrust/self.Kv
        # if abs(u) > U_ERROR:
        #     return 0

        return u/abs(u)*min(abs(u),MAX_VOLTAGE)

class LQR(object):
    def __init__(self):
        self.Q = np.diag([10.0,2.0])
        self.R = np.array([[5.0]])
        # self.desired_poles = (-1.5, -1.33)
    
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

    def pole_placement(self, X, dt, d0):
        z, zdot, *_ = X
        V = 0.0514 
        Kv = 0.714
        Dz = 0.0480
        Xi = [z,zdot,V,Kv,Dz]

        m = M_chassis + RHO_HE*V + M_AZ
        m_RB = m - M_AZ

        Xc = np.array(X[:2]).reshape((2,1))
        goal = np.array([[1.0],[0.0]])

        Fb = (RHO_AIR-RHO_HE)*g*V
        u0 = m_RB*g - Fb

        A, B = self.get_AB(Xi, dt, m)

        K = place_poles(A,B,self.desired_poles).gain_matrix

        thrust = u0 - K@(Xc - goal)
        d = d0 + thrust/Kv
        
        return d/abs(d)*min(abs(d),MAX_VOLTAGE)

    def get_control(self, X, dt, d0):
        z, zdot, V, Kv, Cd = X
        V = 0.0514 
        Kv = 0.714
        Dz = 0.0480
        # v = 0.0514 
        # V = 0.010816676121561977
        # Kv = 0.4589
        # Cd = 0.6227
        m = M_chassis + RHO_HE*V + M_AZ
        m_RB = m - M_AZ

        Xc = np.array(X[:2]).reshape((2,1))

        goal = np.array([[1.5],[0.0]])

        Fb = (RHO_AIR-RHO_HE)*g*V
        A, B = self.get_AB(X, dt, m)
        u0 = m_RB*g - Fb

        P = solve_discrete_are(A,B,self.Q,self.R)
        K = np.linalg.inv(B.T @ P @ B + self.R) @ (B.T @ P @ A)
        thrust = u0 - K@(Xc - goal)
        d = d0 + thrust/Kv
        
        return d/abs(d)*min(abs(d),MAX_VOLTAGE)