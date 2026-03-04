import matplotlib.pyplot as plt
import numpy as np

# t = np.loadtxt("times.txt")
# p = np.load("poses.txt")
t = np.loadtxt("times_cbf.txt")
t = t - t[0]

t_len = t.shape[0]

attentions = np.loadtxt("attentions.txt")
biases = np.loadtxt("biases.txt")
opinions = np.loadtxt("opinions.txt")

nod=False
if len(attentions)>0:
    nod=True
    fig, ax = plt.subplots(1,3,figsize=(12,12))
    ax[0].plot(t[1:],attentions)
    ax[0].set_ylabel("Attention")
    ax[1].plot(t[1:],biases)
    ax[1].set_ylabel("Bias")
    ax[2].plot(t[1:],opinions)
    ax[2].set_ylabel("Opinions")
    ax[2].legend(["Blimp 0","Blimp 1"])
    for i in range(3):
        ax[i].grid()
    fig.supxlabel("Time (s)")
    fig.suptitle("NOD Parameters over time")
    plt.tight_layout()
    plt.show()

t_b0 = np.loadtxt("agent_0_times.txt")
poses_b0 = np.loadtxt("agent_0_poses.txt")
goals_b0 = np.loadtxt("agent_0_goals.txt")
t_b0 -= t_b0[0]

t_b1 = np.loadtxt("agent_1_times.txt")
t_b1 -= t_b1[0]
poses_b1 = np.loadtxt("agent_1_poses.txt")
goals_b1 = np.loadtxt("agent_1_goals.txt")

# before_nod = np.loadtxt("before_nod.txt").reshape( (t_len,6) )
nominal = np.loadtxt("nominal_velocities.txt").reshape( (t_len,6) )
adjusted = np.loadtxt("adjusted_velocities.txt").reshape( (t_len,6) )
# last = adjusted[-1].reshape(1,6)
# print(last)
# adjusted = np.concatenate([adjusted,last],axis=0)

distances = np.loadtxt("distances.txt")

b0_nom = nominal[:,0:3]
b0_adj = adjusted[:,0:3]
b1_nom = nominal[:,3:6]
b1_adj = adjusted[:,3:6]

fig,ax = plt.subplots(2,3,figsize=(12,12))
imap = {0:'X',1:'Y',2:'Z'}
s = 'without NOD' if not nod else 'with NOD'
for i in range(3):
    ax[0,i].plot(t,b0_nom[:,i],label=f'u_nom {imap[i]}')
    ax[0,i].plot(t,b0_adj[:,i],label=f'u_adj {imap[i]}')
    ax[1,i].plot(t,b1_nom[:,i],label=f'u_nom {imap[i]}')
    ax[1,i].plot(t,b1_adj[:,i],label=f'u_adj {imap[i]}')
    ax[0,i].legend(loc="upper left",fontsize=8)
    ax[1,i].legend(loc="upper left",fontsize=8)
    ax[0,i].grid()
    ax[1,i].grid()
    for j in range(2):
        ax[j,i].set_ylim([-2,2])
ax[0,0].set_ylabel("Blimp 0")
ax[1,0].set_ylabel("Blimp 1")
fig.suptitle(f"Nominal vs. Adjusted CBF Velocity {s}")
fig.supxlabel("Time (s)")
fig.supylabel("Desired Velocity (m/s)")
# plt.show()

# fig = plt.figure(figsize=(12,12))
# ax0 = fig.add_subplot(121,projection='3d')
# ax1 = fig.add_subplot(122,projection='3d')
# ax0.plot(poses_b0[:,0],poses_b0[:,1],poses_b0[:,2],label="Trajectory")
# ax1.plot(poses_b1[:,0],poses_b1[:,1],poses_b1[:,2],label="Trajectory")
# ax0.scatter(poses_b0[0,0],poses_b0[0,1],poses_b0[0,2],marker='x',color='red',label="Start")
# ax1.scatter(poses_b1[0,0],poses_b1[0,1],poses_b1[0,2],marker='x',color='red',label="Start")
# ax0.scatter(poses_b0[-1,0],poses_b0[-1,1],poses_b0[-1,2],marker='o',color='red',label="End")
# ax1.scatter(poses_b1[-1,0],poses_b1[-1,1],poses_b1[-1,2],marker='o',color='red',label="End")
# # ax0.plot(goals_b0[:,0],goals_b0[:,1],goals_b0[:,2],color='green',label="Goal")
# # ax1.plot(goals_b1[:,0],goals_b1[:,1],goals_b1[:,2],color='green',label="Goal")
# ax1.legend()
# ax0.legend()
# ax0.set_title("Blimp 0 Trajectory")
# ax0.set_zlabel("Z Position")
# ax0.set_ylabel("Y Position")
# ax0.set_xlabel("X Position")
# ax1.set_title("Blimp 1 Trajectory")
# ax1.set_zlabel("Z Position")
# ax1.set_ylabel("Y Position")
# ax1.set_xlabel("X Position")
# # ax0.view_init(elev=180, azim=55)
# # ax1.view_init(elev=180, azim=55)
# plt.show()

fig, ax = plt.subplots(2,2,figsize=(12,12))
ax[0,0].plot(poses_b0[:,0],poses_b0[:,1],label="Trajectory")
ax[0,0].scatter(poses_b0[0,0],poses_b0[0,1],marker="x",color="red",label="Start")
ax[0,0].scatter(poses_b0[-1,0],poses_b0[-1,1],marker="o",color="green",label="End")
ax[0,0].set_xlabel("X Position")
ax[0,0].set_ylabel("Y Position")
ax[0,1].plot(poses_b1[:,0],poses_b1[:,1],label="Trajectory")
ax[0,1].scatter(poses_b1[0,0],poses_b1[0,1],marker="x",color="red",label="Start")
ax[0,1].scatter(poses_b1[-1,0],poses_b1[-1,1],marker="o",color="green",label="End")
ax[0,1].set_xlabel("X Position")
ax[0,1].set_ylabel("Y Position")
ax[1,0].plot(t_b0,poses_b0[:,2],label="Trajectory")
ax[1,0].set_xlabel("Time (s)")
ax[1,0].set_ylabel("Altitude (m)")
ax[1,1].plot(t_b1,poses_b1[:,2],label="Trajectory")
ax[1,1].set_xlabel("Time (s)")
ax[1,1].set_ylabel("Altitude (m)")

x_min = np.min(np.concatenate( (poses_b0[:,0],poses_b1[:,0]) ))
x_max = np.max(np.concatenate( (poses_b0[:,0],poses_b1[:,0]) ))
ax[0,0].set_xlim([x_min-0.5,x_max+0.5])
ax[0,1].set_xlim([x_min-0.5,x_max+0.5])

y_min = np.min(np.concatenate( (poses_b0[:,1],poses_b1[:,1]) ))
y_max = np.max(np.concatenate( (poses_b0[:,1],poses_b1[:,1]) ))
ax[0,0].set_ylim([y_min-0.5,y_max+0.5])
ax[0,1].set_ylim([y_min-0.5,y_max+0.5])

z_min = np.min(np.concatenate( (poses_b0[:,2],poses_b1[:,2]) ))
z_max = np.max(np.concatenate( (poses_b0[:,2],poses_b1[:,2]) ))
ax[1,0].set_ylim([z_min-0.5,z_max+0.5])
ax[1,1].set_ylim([z_min-0.5,z_max+0.5])

ax[0,0].set_title("Blimp 0")
ax[0,1].set_title("Blimp 1")

# d = np.linalg.norm(poses_b0[:,0:2] - poses_b1[:,0:2],axis=0)
# print(d)


for i in range(2):
    for j in range(2):
        ax[i,j].grid()
        ax[i,j].legend(loc="upper left",fontsize="small")

fig.suptitle("Trajectories Over Time")
plt.show()
fig,ax = plt.subplots(2,1)
n0 = np.linalg.norm(poses_b0[:,6:8],axis=1)
t_b0 = t_b0[n0<1.0]
n0 = n0[n0<1.0]
n1 = np.linalg.norm(poses_b1[:,6:8],axis=1)
t_b1 = t_b1[n1<1.0]
n1 = n1[n1<1.0]
ax[0].plot(t_b0,n0)
ax[1].plot(t_b1,n1)
for i in range(2):
    ax[i].set_ylabel("Magnitude of Velocity")
    ax[i].set_xlabel("Time (s)")
    ax[i].set_title(f"Magnitude of Velocity of Blimp {i}")
    ax[i].grid()

plt.show()

plt.plot(t,distances)
plt.axhline(1.0,linestyle='--',color='red',label="Safety Distance")
plt.title(f"Distance between Blimps, {s}")
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.grid()
plt.legend()
plt.show()