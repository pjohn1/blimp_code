#################################################################
# Written for my final project for Multi-Agent Coordination     #
# CBFs simulating the blimps having full knowledge of each      #
# others states; explores different architectures               #
#################################################################

import rclpy
from rclpy.node import Node
from blimp_msgs.msg import GoalMsg, OptiTrackPose

import threading
import numpy as np

from scipy import sparse
import networkx
import osqp

# NOMINAL_V = 1.5
MAX_V = 1.0

class CBF(Node):
    def __init__(self):
        super().__init__('cbf_node')

        self.declare_parameter('agents',['COM7','agent_61'])
        a = self.get_parameter('agents').value
        self.agents = [a[i] for i in range(len(a)) if i%2 != 0]

        self.num_blimps = len(self.agents)

        self.declare_parameter('goals',['agent_63','agent_61'])
        g = self.get_parameter('goals').value
        self.goal_map = {g[i]:g[i+1] for i in range(len(g)) if i%2 == 0}

        self.declare_parameter('lookahead',0.5)
        self.lookahead = self.get_parameter('lookahead').value

        self.poses = {a:None for a in self.agents}
        self.goals = {a:None for a in self.agents}
        self.publishers_ = {a:() for a in self.agents}

        self.declare_parameter('dmin',1.5)
        self.dmin = self.get_parameter('dmin').value

        self.declare_parameter('kappa',1.0)
        self.kappa = self.get_parameter('kappa').value

        self.declare_parameter('k',1.0)
        self.k = self.get_parameter('k').value

        self.declare_parameter('use_nod',True)
        self.use_nod = self.get_parameter('use_nod').value
        
        self.received_goals = False
        self.received_poses = False

        self.nominals = []
        self.adjusted = []
        self.times = []
        self.opinion_states = []
        self.attentions = []
        self.biases = []
        self.distances = []
        self.before_nod = []

        ##### NOD PARAMETERS ##########
        G = networkx.cycle_graph(len(self.agents))
        self.A = lambda i: G.neighbors(i)
        self.d = lambda i: G.degree(i)
        self.nu_low = 0.2
        self.nu_high = 1.2
        self.d_th = self.dmin*1.25
        self.k_i = 0.1
        self.beta = 0.1
        self.p_center = None
        self.b_ = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ]) #90 deg rotation matrix

        self.opinions = None

        self.last_time = None
        self.u0 = None

        ####### QP SOLVER #################
        self.solver = osqp.OSQP(verbose=False)
        ###################################

        for a in self.agents:
            self.create_subscription(OptiTrackPose,f'/{a}/optitrack_node/pose',self.run_cbf,5)
            self.create_subscription(GoalMsg,f'/{a}/high_level_controller/goal',self.update_goals,5)
            self.publishers_[a] = self.create_publisher(GoalMsg,f'/{a}/controller/goal',5)

    
    def update_goals(self,msg):
        id_ = msg.id
        string_id = f'agent_{id_}'
        self.goals[self.goal_map[string_id]] = np.array([msg.x,msg.y,1.75])#,msg.roll,msg.pitch,msg.yaw])#,msg.ux,msg.uy,msg.uz,msg.wx,msg.wy,msg.wz])
        if not self.received_goals and all([self.goals[i] is not None for i in self.goals]):
            self.received_goals = True
        if self.received_goals:
            self.p_center = np.mean(np.array(list(self.goals.values())),axis=0)
    
    def proj_onto_halfspace(self,a,b,u):
        '''
        Projects u onto the halfspace determined by a and b

        code based on HW4 skeleton code
        '''
        at_u = float(a @ u)
        if at_u >= b:
            return u
        
        a_norm2 = float(a @ a)
        if a_norm2 < 1e-12:
            return u
        return u + ((b-at_u)/a_norm2)*a
    
    def compute_attention_and_bias(self,positions,u_qp,A,nu_low,nu_high,d_th,k_i,p_center):
        #pc is 0
        N = self.num_blimps
        nu_bar = 1/N*sum([np.linalg.norm(i) for i in u_qp])
        self.get_logger().info(f'nu_bar: {nu_bar}')
        # nu_t = nu_low if nu_bar > v_th else nu_high
        biases = []
        for i in range(N):
            ni_l = ni_r = 0
            for j in A(i):
                self.get_logger().info(f'{j}')
                pi = positions[f'agent_{i}']
                pj = positions[f'agent_{j}']

                d = pj-pi
                self.get_logger().info(f'distance: {np.linalg.norm(d)}')
                if np.linalg.norm(d) < d_th:
                    nu_t = nu_high
                else:
                    nu_t = nu_low
                ni = (pi - p_center)/np.linalg.norm(pi - p_center)
                bi = self.b_@ni

                dp = np.dot(bi,d)
                
                bi = np.reshape(bi,(1,3))
                d = np.reshape(d,(1,3))

                sij = (1/2 * np.cross(bi,d))[0]

                if sij[-1] > 0:
                    ni_l += 1
                elif sij[-1] < 0:
                    ni_r += 1

            bias = k_i*(ni_r - ni_l)
            biases.append(bias)
        return nu_t,biases

    def update_opinion_state(self,x,A,d,nu_t,I,dt):
        N = self.num_blimps
        for i in range(N):
            inner_sum = 0
            for j in A(i):
                inner_sum += x[j] #uniform weights, a_ij=1
            xdot = -x[i] + np.tanh(nu_t*(x[i] + inner_sum)) + I[i]
            x[i] += xdot*dt
        return x
    
    def compute_nominal_control_with_NOD(self,p,p_star,x,p_center,k,beta):
        N=self.num_blimps
        u_nod = np.zeros((N,3))
        u0 = [np.clip(self.k*(self.goals[a] - self.poses[a]),-MAX_V,MAX_V) for a in self.agents]
        for i in range(N):
            ni = (p[f'agent_{i}'] - p_center)/np.linalg.norm(p[f'agent_{i}']-p_center)
            b_i = self.b_ @ ni
            u_nod[i] = u0[i] + beta*x[i]*b_i

        return u_nod

    def compute_nominal_from_nod(self,dt):
        nu_t, I = self.compute_attention_and_bias(self.poses,self.u0,self.A,self.nu_low,self.nu_high,self.d_th,self.k_i,self.p_center)
        self.opinions = self.update_opinion_state(self.opinions,self.A,self.d,nu_t,I,dt)
        self.opinion_states.append(self.opinions)
        self.attentions.append(nu_t)
        self.biases.append(I)
        u0 = self.compute_nominal_control_with_NOD(self.poses,self.goals,self.opinions,self.p_center,self.k,self.beta)
        return u0


    def half_plane_cbf(self):
        u = np.zeros( (len(self.agents),3) )
        for i in range(len(self.agents)):
            min_u = (float('-inf'),None)
            for j in range(len(self.agents)):
                if i!=j:
                    a1,a2 = (self.agents[i],self.agents[j])
                    d = (self.poses[a1] - self.poses[a2])[0:3]
                    hij = -self.kappa * (np.linalg.norm(d)**2)
                    a = 2*d.T
                    b = hij + a@self.u0[j]
                    u_proj = self.proj_onto_halfspace(a,b,self.u0[i])
                    if np.linalg.norm(u_proj - self.u0[i]) > min_u[0]:
                        #if the current projection is larger than the minimum projections
                        #the minimum projection violates a previous constraint
                        #so we keep that maximum of the minimum projections onto the safe space constraints
                        min_u = (np.linalg.norm(u_proj-self.u0[i]),u_proj)
            
            u_ = list(min_u[1])
            u[i] = u_
            return u

    def half_plane_matrices(self,u0_i,u0_j,d):
        aij = 2*d
        b = -self.kappa*(np.linalg.norm(d)**2 - self.dmin**2) + aij@u0_j#1/2*aij@(u0_i+u0_j)
        return aij, b

    def ellipse_matrices(self,u0_i,u0_j,d):
        rx,ry,rz = (1.0*self.dmin,0.5*self.dmin,2.0*self.dmin)
        P_el = np.diag( (1/rx,1/ry,1/rz) )
        aij = (2*d@P_el).T
        b = -self.kappa* (d.T@P_el@d - 1) + aij@(1/2*(u0_i+u0_j)) #reciprocal constraint
        # b = -self.kappa* (d.T@P_el@d - 1) + aij@u0_j
        return aij,b

    def solve_qp_symm(self,u0, matrix_function):
        m = self.num_blimps-1
        # P_el = np.eye(3)/(4*self.dmin)
        rx,ry,rz = (1*self.dmin,1*self.dmin,1*self.dmin)
        P_el = np.diag( (1/rx,1/ry,1/rz) )
        n=3*self.num_blimps
        u_min = -float('inf')*np.ones((n,))
        u_ = np.zeros( (self.num_blimps,3) )

        A = np.empty( (m,n) )
        B = np.empty( (m,m) )

        d = self.poses[f'agent_0'] - self.poses[f'agent_1']
        self.distances.append(np.linalg.norm(d))
        a, b = matrix_function(u0[0],u0[1],d) 
        A[0][0:3] = -a
        A[0][3:6] = a
        B[0] = b

        A_full = np.vstack([A,np.eye(3*self.num_blimps)]) 
        l = np.concatenate([B.flatten(),u_min])
        upper = np.ones((m+n,))*float('inf')
        P = sparse.csc_matrix(np.eye(n))
        q = -np.array(u0).flatten()
        A_sparse = sparse.csc_matrix(A_full)
        self.solver.setup(A=A_sparse,q=q,l=l,u=upper,P=P,verbose=False)
        res = self.solver.solve()
        u_res = res.x
        for i in range(0,len(u_res),3):
            l = list(u_res[i:i+3])
            u_[int(i//3)] = l
        return u_

    def solve_qp(self,u0,matrix_function):
        '''
        Solves QP one agent at a time
        '''
        m = self.num_blimps-1
        n=3
        u_min = -float('inf')*np.ones((3,))

        u_ = np.zeros( (self.num_blimps,3) )
        for i in range(self.num_blimps):
            A = np.empty((m,3))
            B = np.empty((m))
            for j in range(m):
                if i != j:
                    d = self.poses[f'agent_{i}'] - self.poses[f'agent_{j}']
                    self.distances.append(np.linalg.norm(d))
                    a, b = matrix_function(u0[i],u0[j],-d)
                    A[j] = a
                    B[j] = b


            A_full = np.vstack([A,np.eye(3)]) 
            l = np.concatenate([B,u_min],axis=0)
            upper = np.ones((m+n,))*float('inf')
            P = sparse.csc_matrix(np.eye(n))
            q = -u0[i]
            A_sparse = sparse.csc_matrix(A_full)
            self.solver.setup(A=A_sparse,q=q,l=l,u=upper,P=P,verbose=False,polish=True)
            res = self.solver.solve()
            u_res = res.x
            u_res = list(u_res)
            u_[i] = u_res
        return u_


    def run_cbf(self,msg):
        id_ = msg.id
        string_id = f'agent_{id_}'
        self.poses[string_id] = np.array([msg.x,msg.y,msg.z])
        if self.received_goals and self.received_poses:
            t = self.get_clock().now().nanoseconds/1e9
            if self.u0 is None:
                self.u0 = [np.clip(self.k*(self.goals[a] - self.poses[a]),-MAX_V,MAX_V) for a in self.agents]
            if self.opinions is None:
                # self.opinions = [np.linalg.norm(self.poses[a]-self.goals[a]) for a in self.agents]
                self.opinions = [0.0]*self.num_blimps

            self.times.append(t)
            if self.use_nod and self.last_time is not None:
                self.before_nod.append(self.u0)
                dt = t-self.last_time
                self.u0 = self.compute_nominal_from_nod(dt)
                self.u0 = np.clip(self.u0,-MAX_V,MAX_V)
            else:
                self.u0 = [np.clip(self.k*(self.goals[a] - self.poses[a]),-MAX_V,MAX_V) for a in self.agents]
            # self.get_logger().info(f'nominal u: {self.u0}')
            self.nominals.append(self.u0)

            self.last_time = t

            self.u0 = self.solve_qp(self.u0,self.half_plane_matrices)
            # self.u0 = self.solve_qp_symm(self.u0,self.ellipse_matrices)
            self.adjusted.append(self.u0)
            # self.get_logger().info(f'adjusted u0: {self.u0}')

            for idx,a in enumerate(self.publishers_):
                msg = GoalMsg()
                yaw_goal = np.arctan2(self.u0[idx][1],-self.u0[idx][0])
                if yaw_goal < 0:
                    yaw_goal += 2*np.pi

                if np.linalg.norm(self.u0[idx]) < 1e-3:
                    x,y,z = self.poses[a]
                else:
                    x,y,z = self.poses[a] + np.sign(self.u0[idx])*np.minimum(self.lookahead,np.abs(self.u0[idx]))

                goals = self.goals[a].copy()

                msg.id = int(a.split('_')[-1])
                msg.x = x
                msg.y = y
                msg.z = z
                msg.yaw = yaw_goal
                # msg.ux = u[idx][0]
                # msg.uy = u[idx][1]
                # msg.uz = u[idx][2]
                # self.get_logger().info(f'vx,vy,vz: {msg.ux},{msg.uy},{msg.uz}')
                self.publishers_[a].publish(msg)
        elif not self.received_poses:
            self.received_poses = True if all([self.poses[i] is not None for i in self.poses]) else False
        

def main(args=None):
    rclpy.init(args=args)
    cbf = CBF()
    try:
        rclpy.spin(cbf)
    except KeyboardInterrupt:
        np.savetxt("nominal_velocities.txt",np.array(cbf.nominals).flatten())
        np.savetxt("adjusted_velocities.txt",np.array(cbf.adjusted).flatten())
        np.savetxt("times_cbf.txt",np.array(cbf.times))
        np.savetxt("opinions.txt",np.array(cbf.opinion_states))
        np.savetxt("attentions.txt",np.array(cbf.attentions))
        np.savetxt("biases.txt",np.array(cbf.biases))
        np.savetxt("distances.txt",np.array(cbf.distances))
        np.savetxt("before_nod.txt",np.array(cbf.before_nod))
    cbf.destroy_node()