
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from blimp_msgs.msg import OptiTrackPose, GoalMsg

import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.optimize import lsq_linear
from scipy.signal import cont2discrete

import tinympc
import time

## BLIMP PROPERTIES ##################
# https://doi.org/10.1142/S2301385021500060?urlappend=%3Futm_source%3Dresearchgate.net%26medium%3Darticle

DRONE_PROTOTYPE_2_MASS = 46/1000 #kg
DRONE_PROTOTYPE_0_MASS = 63/1000 #kg
BALLOON_MASS = .0545/2 #kg
m = DRONE_PROTOTYPE_2_MASS + BALLOON_MASS

fb = 9.802 #m/s^2
G = 9.81 #m/s^2
VOLTAGE_CONSTANT = .00714
U_ERROR = 3.0
MAX_VOLTAGE = 0.4
MAX_YAW_VOLTAGE = 0.4
MAX_SK_VOLTAGE = 0.05
Dz = 0#0.0480
Dwy = 0#0.000980
b = .000980

Fb = m*(G-fb)

H_ENV = 80/100 #m
H_GON = 4/100 #m
D_ENV = 81.28/2/100 #m

D_MT = H_ENV/2 + H_GON #distance between CM and gondola
D_VM = 0.0971 #m, found from paper, distance between CM and Center of lift

Icm = 0.005821/2 #Icm, should be roughly correct

###### OPTITRACK PROPERTIES ##########

ALT_OFFSET = 0 #Distance between pivot height and floor
YAW_OFFSET = 0 #Difference between where 0 yaw is and where it is desired
PITCH_OFFSET = -0.05
ROLL_OFFSET = 0#0.1

######################################

####### PROCESSING ###################
N_avg_pts = 1
N_avg_v = 5

# LQR CONSTANTS ######################
Q_alt = np.diag([10.0,5.0])
R_alt = np.array([[3.0]])

# Q_yaw = np.diag([30.0,15.0])
# R_yaw = np.array([[50.0]])

Q_yaw = np.diag([10.0,5.0])
R_yaw = np.array([[30.0]])

Q_pitch = np.diag([10.0,10.0,1.0,1.0])
R_pitch = np.diag([5.0,100.0])

Q_sk = np.diag([10.0,10.0])
R_sk = np.diag([5.0,5.0])
######################################

##### SYSTEM STATES ##################
u0_alt = Fb #want to hover so 0 velocity, thrust = gravity - buoyancy - drag
A_alt = np.array([[0.0, 1.0],[0.0, -Dz]]) # partial wrt vdot is drag constant (assume linear drag)
B_alt = np.array([[0.0,1/m]]).T # partial wrt u is 1/m


u0_yaw = 0.0 #0 angular velocity, torque around z = 0
A_yaw = np.array([[0.0, 1.0],[0.0, -1/Icm*Dwy]]) # partial wrt vdot is drag constant (assume linear drag)
B_yaw = np.array([[0.0,D_MT/Icm]]).T # partial wrt u is 1/m

u0_pitch = [0.0,m*(fb-G)*D_VM/D_MT*np.sin(0)] #0 angular velocity, torque around z = 0
A_pitch = np.array([
    [0,0,1,0],
    [0,0,0,1],
    [0,0,0,0],
    [0,-Fb/Icm*D_VM,0,-b/Icm]
],dtype=np.float64)

B_pitch = np.array([
    [0 , 0],
    [0 , 0],
    [1/m , 0],
    [0 , D_MT/Icm]
],dtype=np.float64)

A_sk = np.array([
    [0,0],
    [0,0]
])

B_sk = np.array([
    [1/m,0],
    [0,1/m]
])

#####################################

class ControllerNode(Node):
    '''
    This class sends controls to each individual blimp
    Each blimp has its own ControllerNode class
    There are a few controller iterations, feel free to use
    whichever is best
    '''
    def __init__(self):

        ## ROS NODE PARAMS #################
        super().__init__("controller_node")
        self.ns = self.get_namespace()
        self.publisher = self.create_publisher(Float32MultiArray,f"{self.ns}/serial/voltages",5)
        self.subscriber = self.create_subscription(GoalMsg,f"{self.ns}/controller/goal",self.update_goal,5)
        self.telemetry = self.create_subscription(OptiTrackPose,f"{self.ns}/optitrack_node/pose",self.controller,5)
        ####################################


        ###### CBF START ##############
        self.start_cbf = True
        self.create_subscription(Int32,f'/start_cbf',self.manage_cbf,5)
        
        ##### CONTROLLERS #############
        # self.alt_controller = LQR(u0_alt,A_alt,B_alt,Q_alt,R_alt,self)
        self.alt_controller = MPC(u0_alt,A_alt,B_alt,Q_alt,R_alt,5,self)
        self.yaw_controller = MPC(u0_yaw,A_yaw,B_yaw,Q_yaw,R_yaw,5,self)
        # self.yaw_controller = LQR(u0_yaw,A_yaw,B_yaw,Q_yaw,R_yaw,self)
        self.pitch_controller = MPC_pitch(u0_pitch,A_pitch,B_pitch,Q_pitch,R_pitch,5,self)
        self.station_keeping = LQR(np.array([0.0,0.0]),A_sk,B_sk,Q_sk,R_sk,self)

        ###### MOTOR MIXER PARAMS ########
        #lowkey not working, ideally just rotation matrices based on current yaw
        self.rotation_matrix = lambda yaw: np.array(
            [
                [-np.cos(yaw),-np.sin(yaw)],
                [-np.sin(yaw),np.cos(yaw)]
            ])
        d = np.deg2rad(45)
        self.u_i = np.array(
            [
                [np.cos(d), np.sin(d)], #FL
                [np.cos(-d), np.sin(-d)], #FR
                [np.cos(3*np.pi/4),  np.sin(3*np.pi/4)],  # BR
                [np.cos(-3*np.pi/4), np.sin(-3*np.pi/4)]  # BL
            ]
        )

        ###### PARAMETERS ################

        self.received_goal = False
        self.xs = []
        self.ys = []
        self.alts = []
        self.yaws = []
        self.pitches = []
        self.ts = []

        self.last_rates = [0.0,0.0,0.0,0.0,0.0] #vx,vy,vz,wy,wz

    def manage_cbf(self,msg):
        # runs the cbf, simple subscriber that starts the calculation
        if msg.data == 1:
            self.start_cbf = True
        else:
            self.start_cbf = False

    def update_goal(self,msg):
        self.received_goal = True
        self.x_goal = np.array([msg.x, msg.ux])
        self.y_goal = np.array([msg.y,msg.uy])
        self.alt_goal = np.array([msg.z,msg.uz])
        self.yaw_goal = np.array([msg.yaw,msg.wz])
        self.pitch_goal = np.array([msg.pitch,msg.wy])

        self.pitch_controller.update_u0([0.0,Fb*D_VM/D_MT*np.sin(msg.pitch)])

    def motor_mixer(self,thrusts,yaw):
        '''
        thrusts: [fx,fy] forces in x and y direction
        returns [m1,m2,m3,m4] thrust from each m_i motor
        '''
        # self.get_logger().info(f'{thrusts}')
        B = self.rotation_matrix(yaw)@np.array(thrusts)
        dp = np.clip(np.dot(self.u_i,B), 0, 0.9 )
        # dp = dp / np.linalg.norm(dp)
        # return dp
        return dp

    def controller(self,msg):
        mtr = [0.0]*6
        if self.received_goal:
            # build measurement matrix
            x,y,alt,roll,pitch,yaw,t = (msg.x,msg.y,msg.z,msg.roll,msg.pitch,msg.yaw,msg.time)

            self.yaws.append(yaw)
            self.pitches.append(pitch)
            self.xs.append(x)
            self.ys.append(y)
            self.alts.append(alt)
            self.ts.append(t)

            if len(self.alts) >= N_avg_pts:
                #only update rates every N msmts to maintain stability
            
                ##### rates #################

                if len(self.pitches) >= N_avg_v:
                    dt = self.ts[-1] - self.ts[0]
                    d = (self.yaws[-1] - self.yaws[0])
                    d = (d + np.pi) % (2 * np.pi) - np.pi

                    vx = (self.xs[-1] - self.xs[0]) / dt #global velocity
                    vy = (self.ys[-1] - self.ys[0]) / dt
                    vz = (self.alts[-1] - self.alts[0]) / dt
                    wy = (self.pitches[-1] - self.pitches[0]) / dt
                    wz = d / dt
                    self.last_rates = [vx,vy,vz,wy,wz]

                    # reset average lists
                    self.alts = []
                    self.yaws = []
                    self.pitches = []
                    self.xs = []
                    self.ys = []
                    self.ts = []
                else:
                    vx,vy,vz,wy,wz = self.last_rates
                ################################

                #### altitude control ############
                alt_u = self.alt_controller.control_output([alt-ALT_OFFSET,vz],self.alt_goal)/2
                mtr[2] = min(alt_u,0.9)
                mtr[3] = max(-alt_u,-0.9)
                ##################################

                ######## euler angle control ############################################
                yaw_u = self.yaw_controller.control_output([yaw-YAW_OFFSET,wz],self.yaw_goal,True)/2
                if abs(yaw_u) > MAX_YAW_VOLTAGE:
                    yaw_u = yaw_u/abs(yaw_u) * MAX_YAW_VOLTAGE


                if abs((yaw-YAW_OFFSET)-self.yaw_goal[0]) < 0.2 and abs(wz) < 0.1 and self.start_cbf: # only fly forward when yaw is stable
                    state = np.array([0.0,pitch-PITCH_OFFSET]) #pose is 0 because goal is relative to the blimp
                    #full state is [x,theta,xdot,thetadot]

                    dist_goal = np.sqrt( self.x_goal[0]**2 + self.y_goal[0]**2)
                    velocity_goal = np.sqrt( self.x_goal[1]**2 + self.y_goal[1]**2 )
                
                    goal = np.array([dist_goal,velocity_goal,self.pitch_goal[0],self.pitch_goal[1]])
                    v = np.sqrt(vx**2 + vy**2)

                    pitch_u = self.pitch_controller.control_output(state,goal,v,wy)/2                    
                    if abs(pitch_u) > MAX_VOLTAGE:
                        pitch_u = pitch_u/abs(pitch_u) * MAX_VOLTAGE

                    if pitch_u > 0:
                        mtr[4] = pitch_u
                        mtr[5] = pitch_u
                    elif pitch_u < 0:
                        mtr[0] = abs(pitch_u)
                        mtr[1] = abs(pitch_u)
                else:
                    if yaw_u > 0:
                        mtr[1] = min(yaw_u,0.3)
                        mtr[4] = min(yaw_u,0.3)
                    elif yaw_u < 0:
                        mtr[0] = min(0.3,abs(yaw_u))
                        mtr[5] = min(0.3,abs(yaw_u))
                
                ###########################################################################


                ################## STATION KEEPING CONTROL #################################
                # self.get_logger().info(f'yaw,goal_yaw: {yaw},{self.yaw_goal[0]}')
                # sk_state = np.array([-vx,vy]) #x is inverted
                # res = self.station_keeping.control_output(sk_state,np.array([0.0,0.0]))/2 #x ctrl, y ctrl
                # # self.get_logger().info(f'{res}')
                # if not isinstance(res,float):
                #     ux,uy = res
                #     if abs(ux) > MAX_SK_VOLTAGE:
                #         ux = ux/abs(ux)*MAX_SK_VOLTAGE
                #     if abs(uy) > MAX_SK_VOLTAGE:
                #         uy = uy/abs(uy)*MAX_SK_VOLTAGE

                #     forces = self.motor_mixer([ux,0.0],yaw)
                #     forces += self.motor_mixer([0.0,uy],yaw)

                #     forces = np.clip(forces,0.0,MAX_SK_VOLTAGE)
                #     mtr[0] += forces[0]
                #     mtr[1] += forces[1]
                #     mtr[2] += forces[2]
                #     mtr[3] += forces[3]
                ###############################################################################

                # Publish message        
                t = self.get_clock().now().nanoseconds/1e9

                msg = Float32MultiArray()
                msg.data = list(np.array(mtr).astype(float))
                msg.data.append(float(self.ns.split('_')[-1]))
                self.publisher.publish(msg)


class LQR(object):
    '''
    Basic LQR Compnesator
    Pros: Highly efficient
    Cons: Constant gains
    '''
    def __init__(self,u0,A,B,Q,R,node):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.u0 = u0
        P = solve_continuous_are(self.A,self.B,self.Q,self.R)
        self.K = np.linalg.inv(self.R)@self.B.T@P

        self.node = node

        self.thrust_to_voltage = lambda thrust: thrust/abs(thrust) * np.sqrt(abs(thrust))/VOLTAGE_CONSTANT/100

        self.last_t = None
        self.last_control = 0
        self.last_pose = None
        self.poses = []
        self.velocities = []
    
    def control_output(self,pose,goal,wrap=False):

        state = np.array([pose[0],pose[1]])

        if wrap:
            d = goal[0]-state[0]
            d = (d + np.pi) % (2 * np.pi) - np.pi
            goal = np.array([d,goal[1]])
            state[0] = 0.0

        thrust = (self.u0 - self.K@(state-goal))

        u = self.thrust_to_voltage(thrust)
        if np.any(abs(u) > U_ERROR):
            return 0
        return u
        
    def update_u0(self,new_u0):
        self.u0 = new_u0

class MPC(object):
    '''
    MPC Control using tinyMPC
    Pros: Fast, Very effective
    Cons: Not realistic for decentralized system with cheap hardware
    '''
    def __init__(self,u0,A,B,Q,R,N,node):

        self.node = node
        #Discretize continuous A,B matrices
        dt = 1/(200/N_avg_v)
        Ad, Bd, _, _, _ = \
                    cont2discrete((A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], B.shape[1]))), dt, method='zoh')

        self.solver = tinympc.TinyMPC()
        # self.solver.set_x_ref([self.node.x_goal[0],self.node.pitch_goal[0],self.node.x_goal[1],self.pitch_goal[1]])
        self.u0 = [u0]
        self.solver.setup(Ad,Bd,Q,R,N,verbose=False)
        self.solver.set_u_ref(np.array(self.u0))
        

        self.thrust_to_voltage = lambda thrust: thrust/abs(thrust) * np.sqrt(abs(thrust))/VOLTAGE_CONSTANT/100

    def update_u0(self,new_u0):
        self.u0 = new_u0

    def control_output(self,pose,goal,wrap=False):

        state = np.array(pose)
        local_goal = goal.copy() # dont change goal array

        if wrap: #ensure that the distance to target is minimum distance
            d = local_goal[0] - state[0]
            d = (d + np.pi) % (2 * np.pi) - np.pi
            state[0] = 0.0 #goal is now relative to current state so set=0
            local_goal[0] = d

        self.solver.set_x0(state)
        self.solver.set_u_ref(np.array(self.u0))
        self.solver.set_x_ref(local_goal)

        thrust = self.solver.solve()['controls'][0]
        u = self.thrust_to_voltage(thrust)
        if abs(u) > U_ERROR:
            return 0

        return u

class MPC_pitch(object):
    '''
    MPC but pitch relies on more states
    '''
    def __init__(self,u0,A,B,Q,R,N,node):

        self.node = node
        #Discretize continuous A,B matrices
        dt = 0.02
        Ad, Bd, _, _, _ = \
                    cont2discrete((A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], B.shape[1]))), dt, method='zoh')

        self.solver = tinympc.TinyMPC()
        # self.solver.set_x_ref([self.node.x_goal[0],self.node.pitch_goal[0],self.node.x_goal[1],self.pitch_goal[1]])
        self.u0 = u0
        self.solver.setup(Ad,Bd,Q,R,N,rho=1.0,verbose=False)
        self.solver.set_u_ref(np.array(self.u0))

        self.thrust_to_voltage = lambda thrust: thrust/abs(thrust) * np.sqrt(abs(thrust))/VOLTAGE_CONSTANT/100

    def update_u0(self,new_u0):
        self.u0 = new_u0

    def control_output(self,pose,goal,v,wy):

        d = pose[0] #poses are much more stable than velocities
        theta = pose[1]

        state = np.array([d,theta,v,wy])

        self.solver.set_x0(state)
        self.solver.set_u_ref(np.array(self.u0))
        self.solver.set_x_ref(goal)

        thrust = self.solver.solve()['controls']
        thrust = thrust[0]

        u = self.thrust_to_voltage(thrust)
        if abs(u) > U_ERROR:
            return 0

        self.last_control = u
        return u



def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
if __name__ == '__main__':
    main()