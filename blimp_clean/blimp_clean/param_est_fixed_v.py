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

'''
Things done:
- Create a simulation to ensure that the logic was functioning
- Made an inital parameter estimator using real-world estimates
- Implemented an MPC controller via estimates to see performance
- Measurement gate via test statistic
- Used NIS to tune Q and R based on the consistency
    - Decreased the variance of R when NIS was consistently outside the bounds
    -
'''



RHO_AIR = 1.225 #kg/m^3
RHO_HE = 0.165 # kg/m^3
M_chassis = 46/1000 #kg
M_AZ = 0.0311 #kg
g = 9.81

# 50 cm tall
# 91.44 cm across

# V is now treated as a known constant (not estimated). State is [z, zdot, Kv, Cd].
V = 0.0508
Kv_guess = 0.714
Cd_guess = 0.0100

# V = 0.05534
# Kv_guess = 0.004782
# Cd_guess = 0.01108


MAX_VOLTAGE = 0.8



class Model(object):
    def __init__(self, d0=0.0):
        '''
        Initialize model update class

        '''
        self.d0 = d0
        self.m = M_chassis + RHO_HE*V + M_AZ
        self.m_RB = self.m - M_AZ

    def get_FG(self,X, d, dt, d0):

        z, zdot, Kv, Cd = X

        u = d

        # Make F and G. State order: [z, zdot, Kv, Cd].
        # Dynamics: m*v_dot = (RHO_AIR-RHO_HE)*g*V - M_chassis*g + 2*Kv*(u-d0) - Cd*zdot
        # so dv_next/dKv = 2*(u-d0)*dt/m and dv_next/dCd = -zdot*dt/m.
        F = np.array([
            [1, dt, 0, 0],
            [0, 1 - 1/self.m * Cd * dt, 2/self.m*(u-d0)*dt, -zdot/self.m*dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        # G maps thrust-voltage process noise into the state. Thrust enters as 2*Kv*u,
        # so the velocity row gets 2*Kv*dt/m, matching the F entry.
        G = np.array([
            [0],
            [2/self.m*Kv*dt],
            [0],
            [0],
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
        self.measure_v = False



        self.d0 = 0.0
        self.N_avg = 1
        self.N_ts = 15
        self.N_update_goal = 10

        self.count_measurements = 0

        self.X = [0.0, 0.0, Kv_guess, Cd_guess] # Tracks the current position and parameters
        self.model = Model()

        #Initialize controller
        # self.controller = LQR()
        self.controller = MPC(5,self)
        self.alt_goal = 1.5

        # Tracking
        self.last_time = None
        self.last_z = None
        self.update_lock = threading.Lock()
        self.saved = False

        # Plotting
        self.state_estimates = []
        self.covariance_estimates = []
        self.test_statistics = []
        self.avg_test_statistics = []
        self.innovations = []

        self.last_P = None

        self.init_params()

    def init_params(self):
        if self.measure_v:
            self.H = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ])

            self.R = np.diag([0.001**2,0.075**2]) # Uncertainty in measurements
        else:
            self.H = np.array([[1, 0, 0, 0]])
            self.R = np.array([[0.001**2]])

        # Thrust process noise (enters via G into the velocity row)
        self.Q = np.eye(1)*0.1**2
        # Parameter random walk so the EKF stays alive on Kv, Cd. Without this,
        # P[Kv,Kv] and P[Cd,Cd] only ever shrink and the filter freezes before convergence.
        self.Q_param = np.diag([0.0, 0.0, 1e-6, 1e-8])
        self.P = np.diag([0.1**2, 0.5**2, 0.2**2, 0.1**2]) # Uncertainty in estimate (z, zdot, Kv, Cd)



    def start(self,msg):
        self.run_calibration = msg.data

    def ekf(self, msg):
        if not self.finished_calibration:
            # if len(self.zs)==self.N_avg:
            if self.last_z is not None:
                self.count_measurements += 1
                dt = msg.time - self.last_time
                v = (msg.z - self.last_z) / dt
                if abs(v) > 1.5:
                    return

                if self.measure_v:
                    measurement = np.array([msg.z,v])
                else:
                    measurement = msg.z

                self.X[0] = msg.z
                self.X[1] = v


                if self.run_calibration:
                    with self.update_lock:
                        t0 = time.time()
                        
                        #Update goal every 10 msmts to get some level of velocity
                        if self.count_measurements % self.N_update_goal == 0:
                            self.alt_goal = np.random.uniform(1.0, 2.0)

                        # Get voltage output
                        u = self.controller.get_control(self.X, np.array([self.alt_goal, 0.0]))
                        run_ekf = True
                        if run_ekf:

                            Xk = np.array(self.X.copy())
                            x, xdot, Kv, Cd = Xk

                            # Predict
                            Xnext = Xk.copy()
                            Xnext[0] = Xk[1] # velocity
                            Xnext[1] =  1/self.model.m * ( (RHO_AIR-RHO_HE)*g*V - self.model.m_RB*g + 2*Kv*u - Cd*Xk[1] ) # acceleration
                            Xnext[2] = 0 # parameters are constant
                            Xnext[3] = 0

                            Xp = Xk + dt*Xnext # Predicted state
                            F, G = self.model.get_FG(Xp, u, dt,self.d0)
                            Pp = F@self.P@F.T + G@self.Q@G.T #Predicted covariance

                            # Kalman gain
                            S = self.H@Pp@self.H.T + self.R
                            K = Pp@self.H.T@np.linalg.inv(S)

                            # Innovation
                            nu = measurement - self.H@Xp
                            test_statistic = nu.T @ np.linalg.inv(S) @ nu
                            # INSERT_YOUR_CODE
                            # 95% chi2inv with 2 measurements
                            if self.measure_v:
                                chi2_threshold = 5.991  # For 2 DOF, 95% confidence interval
                            else:
                                chi2_threshold = 3.841

                    
                            if test_statistic < chi2_threshold:
                                self.innovations.append(nu.squeeze())

                                #Update
                                self.X = Xp + K @ nu
                                self.P = (np.eye(4) - K@self.H)@Pp@(np.eye(4)-K@self.H).T + K@self.R@K.T

                                self.X[2] = max(1e-6, self.X[2])
                                self.X[3] = max(1e-6, self.X[3])

                                self.state_estimates.append(self.X)
                                self.covariance_estimates.append(self.P)
                                self.test_statistics.append(test_statistic)

                                if len(self.test_statistics) > self.N_ts:
                                    NIS = np.mean(self.test_statistics[-self.N_ts:])
                                    self.avg_test_statistics.append(NIS)

                                if len(self.innovations) > self.N_ts:
                                    window = self.innovations[-self.N_ts:]
                                    
                                    mean_inn = np.mean(window)
                                    autocorr = np.corrcoef(window[:-1], window[1:])[0,1]

                                    self.get_logger().info(f'Mean innovation: {mean_inn}, Autocorrelation: {autocorr}')

                                    if abs(mean_inn) < 1e-3 and abs(autocorr) < 0.05:
                                        self.finished_calibration = True
                                        self.finished_time = msg.time
                                        self.get_logger().info(f'Estimated Kv: {self.X[2]}, Cd: {self.X[3]}')                        

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
        # else:
        #     self.zs.append(msg.z)
        #     self.times.append(msg.time)
        
        else:
            if not self.saved:
                os.makedirs(self.ns, exist_ok=True)
                self.get_logger().info(f'Saving... {len(self.avg_test_statistics)}')
                np.save(self.ns + '/state_estimates.npy', self.state_estimates)
                np.save(self.ns + '/covariance_estimates.npy', self.covariance_estimates)
                np.save(self.ns + '/finished_time.npy', self.finished_time)
                np.save(self.ns + '/test_statistics.npy', self.test_statistics)
                np.save(self.ns + '/avg_test_statistics.npy', self.avg_test_statistics)
                self.saved = True

        self.last_time = msg.time
        self.last_z = msg.z

        self.last_time = msg.time
        self.last_z = msg.z


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
        m = M_chassis + RHO_HE*V + M_AZ
        m_RB = m - M_AZ

        A = np.array([
            [1, dt],
            [0, 1 - Cd_guess/m*dt]
        ])
        B = np.array([
            [0],
            [dt/m]
        ])

        Fb = (RHO_AIR-RHO_HE)*g*V
        self.node.get_logger().info(f'Estimated net buoyancy force: {Fb}')
        u0 = M_chassis*g - Fb
        self.node.get_logger().info(f'Diff between gravity and buoyancy: {u0}')

        Q_alt = np.diag([10.0,5.0])
        R_alt = np.array([[1.0]])

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

        u = thrust/Kv_guess

        return u/abs(u)*min(abs(u),MAX_VOLTAGE)
