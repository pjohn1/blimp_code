import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np


class UpdateControls(Node):
    def __init__(self):
        super().__init__("update_states")
        self.publisher = self.create_publisher(
            Float32MultiArray, "/controls/control_update", 5
        )
        self.subscriber = self.create_subscription(
            Float32MultiArray, "/sim/robot_pos", self.update_control, 5
        )

        self.declare_parameter("num_blimps", 2)
        self.num_blimps = int(self.get_parameter("num_blimps").value)

        self.rng = np.random.default_rng()

    def update_control(self, _msg):
        controls = []
        for _ in range(self.num_blimps):
            # First two motors are bidirectional vertical thrusters.
            vertical = self.rng.uniform(-1.0, 1.0, size=2)
            # Remaining four motors are unidirectional corner thrusters.
            corners = self.rng.uniform(0.0, 1.0, size=4)
            controls.extend(vertical.tolist() + corners.tolist())

        out = Float32MultiArray()
        out.data = controls
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    update_controls = UpdateControls()
    try:
        rclpy.spin(update_controls)
    except KeyboardInterrupt:
        pass
    update_controls.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import numpy as np
from scipy.spatial.transform import Rotation as R
import transforms3d.euler as euler

import networkx
import time
import osqp
from scipy import sparse

class UpdateControls(Node):
    def __init__(self):
        super().__init__('update_states')
        self.publisher = self.create_publisher(Float32MultiArray,'/controls/control_update',5)
        self.subscriber = self.create_subscription(Float32MultiArray,'/sim/robot_pos',self.update_control,5)
        self.declare_parameter('num_blimps',2)
        self.num_blimps = self.get_parameter('num_blimps').value

        self.declare_parameter('blimp_goals',[0.0,0.0,0.0]*self.num_blimps)
        self.blimp_goals = self.get_parameter('blimp_goals').value
        self.get_logger().info(f'Blimp goals: {self.blimp_goals}')
    
        self.declare_parameter('colors',[0.0,0.0,0.0]*self.num_blimps)
        colors = self.get_parameter('colors').value

        self.declare_parameter('blimp_positions',[0.0,0.0,0.0,0.0,0.0,0.0]*self.num_blimps)
        self.blimp_positions = self.get_parameter('blimp_positions').value

        self.declare_parameter('p_center',[0.0,0.0,0.0])
        self.p_center = np.array(self.get_parameter('p_center').value)

        # self.get_logger().info(f'Blimp initial positions: {self.initial_positions}')
        self.paths = [(MarkerArray(), colors[i*3:i*3+3]) for i in range(self.num_blimps)]

        self.goal_publisher = self.create_publisher(MarkerArray,'/control/goal_publish',1)
        self.traj_publisher = self.create_publisher(MarkerArray,'/control/traj_publish',5)
        self.barrier_publisher = self.create_publisher(Marker,'/control/barrier',5)

        self.declare_parameter('dmin',1.5)
        self.dmin = self.get_parameter('dmin').value

        self.declare_parameter('kappa',2.0)
        self.kappa = self.get_parameter('kappa').value

        self.declare_parameter('dt',0.05)
        self.dt = self.get_parameter('dt').value

        self.count = 0
        self.goal_published = False
        self.u0 = None

        ####### NOD PARAMETERS #############################
        self.with_nod = False
        self.G = networkx.cycle_graph(self.num_blimps)
        self.A = lambda n: self.G.neighbors(n)
        self.deg = lambda n: self.G.degree(n)
        self.nu_low = 0.2
        self.nu_high = 1.2
        self.d_th = self.dmin*1.25
        self.k_i = 0.1
        self.beta = 0.1
        self.k = 0.3
        self.x0 = None

        self.b_ = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 1]
        ]) #90 deg rotation matrix
        ####################################################

        ########### PLOTTING ###############################
        self.nominal_controls = []
        self.adjusted_controls = []
        self.time = []
        self.poses = []
        self.distances = []
        self.attentions = []
        self.biases = []
        self.opinions = []
        ####################################################

        ########### SOLVER ####################
        self.solver = osqp.OSQP(verbose=False)

        self.last_t = None
    
    def compute_attention_and_bias(self,positions,u_qp,A,nu_low,nu_high,d_th,k_i,p_center):
        #pc is 0
        N = positions.shape[0]
        # nu_bar = 1/N*sum([np.linalg.norm(i) for i in u_qp])
        d = np.linalg.norm(positions[0]-positions[1])
        nu_t = nu_low if d > d_th else nu_high
        biases = []
        for i in range(N):
            ni_l = ni_r = 0
            for j in A(i):
                pi = positions[i]
                pj = positions[j]

                d = pj-pi
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
        N = len(x)
        for i in range(N):
            inner_sum = 0
            for j in A(i):
                inner_sum += x[j] #uniform weights, a_ij=1
            xdot = -d(i)*x[i] + np.tanh(nu_t*(x[i] + inner_sum)) + I[i]
            x[i] += xdot*dt
        return x

    def compute_nominal_control_with_NOD(self,p,p_star,x,p_center,k,beta):
        N=len(p)
        u_nod = np.zeros((N,3))
        for i in range(N):
            ui = -k*(p[i]-p_star[i])
            ni = (p[i] - p_center)/np.linalg.norm(p[i]-p_center)
            b_i = self.b_ @ ni
            u_nod[i] = ui + beta*x[i]*b_i
        
        return np.array(u_nod)

    def publish_goals(self):
        goals = self.blimp_goals.copy()
        i = 0
        markers = []
        while i < len(goals):
            r,g,b = self.paths[int(i//3)][1]
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.lifetime.sec = 0
            m.pose.position.x = goals[i]
            m.pose.position.y = goals[i+1]
            m.pose.position.z = goals[i+2]
            m.pose.orientation.w = 1.0

            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            markers.append(m)

            i+=3
        msg = MarkerArray()
        msg.markers = markers
        self.goal_publisher.publish(msg)

    def publish_control(self,controls):
        '''
        controls is an nx3 matrix, need to flatten
        '''
        i=0
        u=0
        t = self.get_clock().now().nanoseconds/1e9 
        if self.last_t is not None:
            dt = t - self.last_t
        else:
            dt = 0.0
        self.last_t = t

        if not self.goal_published:
            self.publish_goals()
            self.goal_published = True
        flattened_controls = np.array(controls).flatten()
        if self.count % 20 == 0 and abs(dt)<0.1 and dt >= 0.0:
            while i < len(self.blimp_positions):
                idx = int(i//6)
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.id = np.random.randint(1e6) #ensure no overlap
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.scale.x = 0.1
                m.scale.y = 0.1
                m.scale.z = 0.1
                m.lifetime.sec = 0

                p1 = self.blimp_positions[i:i+3].astype(float)
                v = flattened_controls[u:u+3]
                # p2 = p1 + v
                # d = p2-p1

                # m.points = [Point(x=p1[0],y=p1[1],z=p1[2]),Point(x=p2[0],y=p2[1],z=p2[2])]
                yaw = np.arctan2(v[1],v[0])
                # pitch = np.arctan2(v[2],v[1])
                pitch=0
                roll=0
                m.pose.position.x = p1[0]
                m.pose.position.y = p1[1]
                m.pose.position.z = p1[2]
                qw,qx,qy,qz = euler.euler2quat(roll,pitch,yaw)
                m.pose.orientation.x = qx
                m.pose.orientation.y = qy
                m.pose.orientation.z = qz
                m.pose.orientation.w = qw

                m.color.r = self.paths[idx][1][0]
                m.color.g = self.paths[idx][1][1]
                m.color.b = self.paths[idx][1][2]
                m.color.a = 1.0
                self.paths[idx][0].markers.append(m)
                self.traj_publisher.publish(self.paths[idx][0])

                i+=6
                u+=3
            


        msg = Float32MultiArray()
        msg.data = list(flattened_controls)
        self.publisher.publish(msg)
        self.count+=1

    def pairwise_cbf_halfspaces_for_agent(self,i, X, U_nom, dmin, alpha):
        """
        Build half-space constraints A u_i >= b for agent i:
        2 (p_i - p_j)^T u_i >= -alpha h_ij + 2 (p_i - p_j)^T u_j_nom
        with h_ij = ||p_i - p_j||^2 - dmin^2.
        Returns A (k x 2), b (k,), one row per neighbor j.
        """
        pi = X[i]
        Ai = []
        bi = []
        for j in range(X.shape[0]):
            if j == i:
                continue
            pj = X[j]
            dij = pi - pj
            h_ij = np.dot(dij, dij) - dmin**2
            a = 2.0 * dij                      # row vector acting on u_i
            rhs = -alpha * h_ij + 2.0 * np.dot(dij, U_nom[j])
            Ai.append(a)
            bi.append(rhs)
        if len(Ai) == 0:
            return np.zeros((0,2)), np.zeros((0,))
        return np.vstack(Ai), np.array(bi)


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

    def project_u_nom_via_dykstra(self,u0, A, b, max_cycles=20):
        """
        Dykstra-style cyclic projections onto half-spaces A_k u >= b_k.
        Returns the Euclidean projection of u0 onto the intersection (good approx in few cycles).
        """
        u = u0.copy()
        # Dykstra corrections, one per constraint
        R = np.zeros_like(A)
        if A.shape[0] == 0:
            return u
        for _ in range(max_cycles):
            for k in range(A.shape[0]):
                a = A[k]
                c = b[k]
                y = u + R[k]
                u_new = self.proj_onto_halfspace(y, a, c)
                R[k] = y - u_new
                u = u_new
        return u

    def half_plane_cbf(self,u0):
        u = np.zeros((self.num_blimps,6)) 
        #This runs in polynomial time - solvers would likely run faster, but this is enough for sim
        for i in range(len(self.agent_poses)):
            min_u = (float('-inf'),None)
            for j in range(len(self.agent_poses)):
                if i != j:
                    d = (self.agent_poses[i] - self.agent_poses[j])[0:3]
                    hij = -self.kappa* ( np.linalg.norm(d)**2 - self.dmin**2 )
                    a = 2*d.T
                    b = hij + a@u0[j]
                    u_proj = self.proj_onto_halfspace(a,b,u0[i])
                    # Each u_i is projected orthogonally onto u0 
                    # so will be the minimum u_i-u0 that satisfies that u_j constraint
                    if np.linalg.norm(u_proj - u0[i]) > min_u[0]:
                        #if the current projection is larger than the minimum projections
                        #the minimum projection violates a previous constraint
                        #so we keep that maximum of the minimum projections onto the safe space constraints
                        min_u = (np.linalg.norm(u_proj-u0[i]),u_proj)
            
            u_ = list(min_u[-1])
            u_.extend([0.0,0.0,0.0]) #controls are published as length 6 but for now just controlling velocity
            u[i] = u_
        return u
    

    def half_plane_matrices(self,u0_i,u0_j,d):
        aij = 2*d
        b = -self.kappa*(np.linalg.norm(d)**2 - self.dmin**2) + 1/2*aij@(u0_i+u0_j)
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
        n=3*self.num_blimps
        u_min = -float('inf')*np.ones((n,))
        u_ = np.zeros( (self.num_blimps,3) )

        A = np.empty( (m,n) )
        B = np.empty( (m,m) )

        d = self.agent_poses[0][0:3] - self.agent_poses[1][0:3]
        self.distances.append(np.linalg.norm(d))
        a, b = matrix_function(u0[0][0:3],u0[1][0:3],-d) 
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
                    d = self.agent_poses[i][0:3] - self.agent_poses[j][0:3]
                    self.distances.append(np.linalg.norm(d))
                    a, b = matrix_function(u0[i][0:3],u0[j][0:3],d)
                    A[j] = a
                    B[j] = b


            A_full = np.vstack([A,np.eye(3)]) 
            l = np.concatenate([B,u_min],axis=0)
            upper = np.ones((m+n,))*float('inf')
            P = sparse.csc_matrix(np.eye(n))
            q = -u0[i]
            A_sparse = sparse.csc_matrix(A_full)
            self.solver.setup(A=A_sparse,q=q,l=l,u=upper,P=P,verbose=False,polish=False)
            res = self.solver.solve()
            u_res = res.x
            u_res = list(u_res)
            u_[i] = u_res
        return u_





        A_full = np.vstack([np.reshape(A,(self.num_blimps,n)),np.eye(3*self.num_blimps)]) 
        l = np.concatenate([B.flatten(),u_min])
        upper = np.ones((self.num_blimps*m+n,))*float('inf')
        P = sparse.csc_matrix(np.eye(n))
        q = -u0.flatten()
        A_sparse = sparse.csc_matrix(A_full)
        self.solver.setup(A=A_sparse,q=q,l=l,u=upper,P=P,verbose=False)
        res = self.solver.solve()
        u_res = res.x
        for i in range(0,len(u_res),3):
            l = list(u_res[i:i+3])
            l.extend([0.0,0.0,0.0])
            u_[int(i//3)] = l
        return u_

                


    def update_control(self,msg,barrier=False):
        '''
        '''
        #constant x velocity
        self.blimp_positions = np.array(msg.data)
        self.agent_poses = np.reshape(self.blimp_positions,(self.num_blimps,6))
        goals = np.reshape(self.blimp_goals,(self.num_blimps,3))
        d = self.agent_poses[:,0:3] - goals # distance between goal and position

        p = self.agent_poses[:,0:3]
        if self.u0 is None:
            self.u0 = -self.k * d

        if self.with_nod:
            if self.x0 is None:
                self.x0 = [np.linalg.norm(i) for i in d]
            nu_t, I = self.compute_attention_and_bias(p,self.u0,self.A,self.nu_low,self.nu_high,self.d_th,self.k_i,self.p_center)
            self.attentions.append(nu_t)
            self.biases.append(I)
            self.x0 = self.update_opinion_state(self.x0,self.A,self.deg,nu_t,I,self.dt)
            self.opinions.append(self.x0)
            self.u0 = self.compute_nominal_control_with_NOD(p,goals,self.x0,self.p_center,self.k,self.beta)
        else:
            self.u0 = 2*-d/np.linalg.norm(d) #v magnitude times direction

        self.nominal_controls.append(self.u0)
        self.u0 = self.solve_qp(self.u0,self.half_plane_matrices)
        # self.u0 = self.solve_qp_symm(self.u0,self.ellipse_matrices)
        self.adjusted_controls.append(self.u0[:,0:3])
        self.time.append(self.get_clock().now().nanoseconds/1e9)
        self.poses.append(p)

        u_ = []
        for ctrl in self.u0:
            l = list(np.round(ctrl,3))
            l.extend([0.0,0.0,0.0])
            u_.append(l)
        self.publish_control(u_)

def main(args=None):
    rclpy.init(args=args)
    update_controls = UpdateControls()
    try:
        rclpy.spin(update_controls)
    except KeyboardInterrupt:
        pass
    np.savetxt('sim_poses.txt',np.array(update_controls.poses).flatten())
    np.savetxt('sim_nominal.txt',np.array(update_controls.nominal_controls).flatten())
    np.savetxt('sim_adjusted.txt',np.array(update_controls.adjusted_controls).flatten())
    np.savetxt('sim_time.txt',np.array(update_controls.time))
    np.savetxt('sim_distances.txt',np.array(update_controls.distances))
    np.savetxt('sim_attentions.txt',np.array(update_controls.attentions))
    np.savetxt('sim_biases.txt',np.array(update_controls.biases))
    np.savetxt('sim_opinions.txt',np.array(update_controls.opinions).flatten())
    time.sleep(0.5)
    update_controls.destroy_node()
    rclpy.shutdown()