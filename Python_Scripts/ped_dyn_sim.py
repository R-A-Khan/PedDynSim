import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import ode

 # Define pedestrian flux phi = flux_ped/flux_dt
 # Constant flux of flux_ped pedestrians every flux_dt seconds
 # Initial spacing between pedestrians is v_d*flux_dt
v_d = 1.2;
flux_ped = 4;
flux_dt = 60;


# Simulation time
TF = 60
t_fine = 1/30  # time accuracy

# Model Parameters #############
tau = 0.7
A = 2000
B = 0.08
Aw = 2000
Bw = 0.08
k1 = 1.2 * 10 ** 5
k2 = 2.4 * 10 ** 5
# HSFM Parameters
kd = 500
ko = 1
k1g = 200  # forward group cohesion force strength
k2g = 200  # sideward group cohesion force strength
d_o = 0.5  # sideward maximum distance from the center of mass
d_f = 1  # forward maximum distance from the center of mass
alpha = 3

# Initial conditions
# Number of individuals in each group
# Define n_i the number of individuals in group i, then
# n_groups = [n_1, n_2, ..., n_N];
# % Individuals are treated as groups of 1 with no group force
# % For simulation time of 60 seconds
# % need constant flux of flux_ped pedestrians every flux_dt seconds
# % in A-B direction and C-D direction
num_groups = int(60/flux_dt *2); 

if flux_ped ==1:
    # % single pedestrian in group
    include_groups = False
else:
    include_groups = True

# % Creating n_groups vector

n_groups = [None]*num_groups 
for k in np.arange(0,num_groups):
    n_groups[k]=int(flux_ped)


N = int(sum(n_groups))

# s{i} contains the starting point of group i
s = {}
# s[0] = [2.5, 0]
# s[1] = [2.5, 25]

xg1_ab = 12.5
yg1_ab = 0

xg1_cd = 0
yg1_cd = 12.5

s[0] = [xg1_ab, yg1_ab];
for i in np.arange(1 , num_groups/2):
    s[i] = [xg1_ab, yg1_ab-(v_d*flux_dt)*(i)];


s[num_groups/2] = [xg1_cd, yg1_cd];
i = 0;
for j in np.arange(num_groups/2 +1 ,num_groups):
    i = i+1
    s[j] = [xg1_cd-(v_d*flux_dt)*(i), yg1_cd];



# waypoints sequence
e_seq = {}
# e_seq{i} contains the points through which the members of group i have to pass
# e_seq[0] = np.array([s[0], [4, 10], [2.5, 25]]).transpose()
# e_seq[1] = np.array([s[1], [1, 10],  [2.5, 0]]).transpose()



for i in np.arange(0,num_groups/2):                 #Corridor AB
    e_seq[i] = np.array([s[i], [13, 20], [12.5, 1000]]).transpose()

for j in np.arange(num_groups/2, num_groups ):              #Corridor CD
    e_seq[j] = np.array([s[j], [20, 12], [1000, 12.5]]).transpose()


e_n = {}        # Number of waypoints
e_ind = {}      # auxiliary index

for i in range(len(n_groups)):
    aux, e_n[i] = e_seq[i].shape                        # number of waypoints of group i
    e_ind[i] = np.zeros((int(n_groups[i]), 1), dtype=int)    # current waypoint for group i

# "am" represents the amplitude of the starting zone. Fix the starting
# points at least at "am" meters from the walls
am = 2

# Individual characteristics
# Radius
rm = 0.25  # minimum radius
rM = 0.25  # maximum radius
# Mass
mm = 90  # minimum mass
mM = 90  # maximum mass
# Desired speed
v0m = 1  # minimum speed
v0M = 1.3  # maximum speed

# Initialize model ##########################################################




def initialization(n_groups, N, rm, rM, mm, mM, v0m, v0M, s, am):
    # Define Map walls ######################
    segments = {}
    # segments[0] = np.array([[0, 25], [25, 25]]).transpose()
    # segments[1] = np.array([[0, 0], [25, 0]]).transpose()
    # # segments_number = segments.__len__()
    
    
    # % %RK: INTERSECTING CORRIDORS
    
    segments[0] = np.array([[10, -60], [10, 10]]).transpose()
    segments[1] = np.array([[15, -60], [15, 10]]).transpose()
    segments[2] = np.array([[-60, 15], [10, 15]]).transpose()
    segments[3] = np.array([[-60, 10], [10, 10]]).transpose()
    segments[4] = np.array([[10, 15], [10, 25]]).transpose()
    segments[5] = np.array([[15, 15], [15, 25]]).transpose()
    segments[6] = np.array([[15, 15], [25, 15]]).transpose()
    segments[7] = np.array([[15, 10], [25, 10]]).transpose()
    
    
    map_walls = np.array([[0,0]])
    for segment in segments:
        map_walls = np.concatenate((map_walls, segments[segment]))
    map_walls = map_walls[1:]
    # Number of walls
    double_num_walls, aux = map_walls.shape
    num_walls = int(double_num_walls / 2)

    v0 = v0m + (v0M - v0m) * np.random.rand(N, 1)  # random desired speed
    v = 0 * np.ones((N, 2))  # initial speed
    th = 2 * np.pi * np.random.rand(N, 1) - np.pi  # initial orientation
    omg = 0  # initial angular velocity

    r = np.empty((N, 1), dtype=float)
    m = np.empty((N, 1), dtype=float)
    group_membership = np.empty((N, 1), dtype=int)
    for i in range(len(n_groups)):  # random radii and masses
        # random radii
        r[sum(n_groups[0: i + 1]) - n_groups[i]: sum(n_groups[0:i + 1])] = np.sort(
            rm + (rM - rm) * np.random.rand(n_groups[i], 1))
        # random masses
        m[sum(n_groups[0: i + 1]) - n_groups[i]: sum(n_groups[0:i + 1])] = np.sort(
            mm + (mM - mm) * np.random.rand(n_groups[i], 1))
        # aux variable
        group_membership[sum(n_groups[0: i + 1]) - n_groups[i]: sum(n_groups[0:i + 1])] = int(i)

    J = 0.5 * r ** 2  # Inertia

    i = 0
    p = {}
    X0 = []
    while i < N:
#         print("This is i: "+str(i))
        gr = int(group_membership[i])
        pos = [s[gr][0] - am + 2 * am * np.random.rand(), s[gr][1] - am + 2 * am * np.random.rand()]
        # minimum distance between pedestrians
        d = []
        for l in range(i):
            d.append(int(np.linalg.norm(pos - np.array(p[l][0:1])) <= r[i] + r[l]))
#             print("This is d before: "+str(d))

        # minimum distance from walls
        for l in range(num_walls):
            # print("This is l: "+str(l))
            xp = pos[0]
            yp = pos[1]
            rp = np.array(pos)
            ra = map_walls[2 * l: 2 * l + 2, 0]
            rb = map_walls[2 * l: 2 * l + 2, 1]
            xa = ra[0]
            ya = ra[1]
            xb = rb[0]
            yb = rb[1]
            t = ((xp - xa) * (xb - xa) + (yp - ya) * (yb - ya)) / (((xb - xa) ** 2 + (yb - ya) ** 2))
            t_star = min(max(0, t), 1)
            rh = ra + t_star * (rb - ra)
            d.append(int(np.linalg.norm(rp - rh) <= r[i]))
#             print("This is d after: "+str(d))
        if sum(d) == 0:
            p[i] = [pos[0], pos[1], v[i, 0], v[i, 1], r[i], m[i]]
            X0 = np.append(X0, [pos[0], pos[1], th[i], np.linalg.norm(v[i, :]), 0, omg])
            i = i + 1

    return map_walls, num_walls, r, m , J, v0, v, th, omg, group_membership, X0, p


# ##############################################################################

map_walls, num_walls, r, m, J, v0, v, th, omg, group_membership, X0, p = initialization(n_groups, N, rm, rM, mm, mM, v0m, v0M, s, am)

# Create class structure to create waypoint sequence
##############################################################################
class waypoints_updater():

    def __init__(self,e_seq, e_n, e_ind, e_act, N, n_groups, group_membership):
        self.e_seq = e_seq
        self.e_n = e_n
        self.e_ind = e_ind
        self.e_act =e_act
        self.N = N
        self.group_membership = group_membership
        self.n_groups = n_groups

    def waypoint_update(self, position, coef):
        e = np.zeros((self.N, 2))
        # Determination of the current waypoints
        for i in range(self.N):
            curr_wayp = self.e_act[int(self.group_membership[i])][i - sum(self.n_groups[0:int(self.group_membership[i])])]
            vect = curr_wayp - position[i]
            vect_norm = np.linalg.norm(curr_wayp - position[i])
            e[i, :] = vect / vect_norm
            current_index = self.e_ind[int(self.group_membership[i])][i - sum(self.n_groups[0:int(self.group_membership[i])])]

            if vect_norm <= coef and current_index < self.e_n[int(self.group_membership[i])]-1:
                current_index += 1
                curr_wayp = self.e_seq[int(self.group_membership[i])][:, current_index].transpose()
                vect = curr_wayp - position[i]
                vect_norm = np.linalg.norm(curr_wayp - position[i])
                e[i, :] = vect / vect_norm

                self.e_ind[int(self.group_membership[i])][i - sum(self.n_groups[0:int(self.group_membership[i])])] = current_index
                self.e_act[int(self.group_membership[i])][i - sum(self.n_groups[0:int(self.group_membership[i])])] = curr_wayp

            if vect_norm <= coef and current_index == self.e_n[int(self.group_membership[i])]:
                e[i, :] = ((1 - np.exp(-5 * vect_norm))/(1+np.exp(-5 * vect_norm))) * (vect / vect_norm)

        return e
 ########################################################################

# Assign the actual position as the current waypoint
e_act = {}
for i in range(len(n_groups)):
    e_act[i] = np.zeros((n_groups[i], 2))
for i in range(N):
    e_act[int(group_membership[i])][i - sum(n_groups[0:int(group_membership[i])])] = p[i][0:2]

waypoints = []
waypoints = waypoints_updater(e_seq, e_n, e_ind, e_act, N, n_groups, group_membership)


# Functions for system definition and forces  ############################

def HSFM_forces(X, e, N, map_walls, num_walls, r, m, v0):
  
    # Positions and velocities
    position = np.zeros((N, 2))
    vel = np.zeros((N, 2))
    for i in range(N):
        position[i, :] = [X[6 * i], X[6 * i+1]]
        vel[i, :] = [X[6 * i+3] * np.cos(X[6 * i+2]), X[6 * i +3] * np.sin(X[6 * i +2])]

    fi0 = np.zeros((N, 2))  # velocity force
    # Interindividual forces
    fij1 = np.zeros((N, 2))   # repulsive
    fij2 = np.zeros((N, 2))   # compression
    fij3 = np.zeros((N, 2))   # friction
    # Obstacles
    fiw1 = np.zeros((N, 2))   # repulsive
    fiw2 = np.zeros((N, 2))   # compression
    fiw3 = np.zeros((N, 2))   # friction
    ang = np.zeros((N,1))
    for i in range(N):
        fi0[i,:] = m[i] * (v0[i] * e[i,:] - vel[i,:]) / tau
        vect = e[i,:]
        ang[i] = np.arctan2(vect[1], vect[0])
        for j in range(N):
            if i != j:
                rij = r[i] + r[j]
                dij = np.linalg.norm(position[i] - position[j])
                nij = (position[i] - position[j]) / dij
                fij1[i] = fij1[i] + A * np.exp((rij - dij) / B) * nij
                if dij < rij:
                    fij2[i] = fij2[i] + k1 * (rij - dij) * nij
                    tij = np.array([-nij[1], nij[0]])
                    dvij = np.dot((vel[j] - vel[i]), tij)
                    fij3[i] = fij3[i] + k2 * (rij - dij) * dvij * tij

            # Walls forces
            for w in range(num_walls):
                xp = position[i,0]
                yp = position[i,1]
                rp = np.array(position[i])
                ra = max([map_walls[2*w, 0], map_walls[2*w + 1, 0]], [map_walls[2*w, 1], map_walls[2*w + 1,1]])
                ra = np.array(ra)
                rb = max([map_walls[2*w, 0], map_walls[2*w + 1, 0]], [map_walls[2*w, 1], map_walls[2*w + 1,1]])
                rb = np.array(rb)
                xa = ra[0]
                ya = ra[1]
                xb = rb[0]
                yb = rb[1]
                # a point on AB can be parametrized as s(t)=ra+t(tb-ta), t in [0,1]
                # distance from s to p is phi(t)=||s(t)-p||
                # d(phi^2) gives the t which minimizes the distance from p to the
                # line in which AB lives. Since t in [0,1], t_star=min(max(0,t),1);
                # and the distance from p to AB is ||s(t_star)-p||

                t = ((xp - xa) * (xb - xa) + (yp - ya) * (yb - ya)) / ((xb - xa) ** 2 + (yb - ya) ** 2)
                t_star = min(max(0, t), 1)
                rh = ra + t_star * (rb - ra)
                diw = np.linalg.norm(rp - rh)
                niw = (rp - rh) / diw
                tiw = np.array([-niw[0], niw[1]])
                fiw1[i] = fiw1[i] + Aw * np.exp((r[i] - diw) / Bw) * niw
                if diw < r[i]:
                    fiw2[i] = fiw2[i] + k1 * (r[i] - diw) * niw
                    fiw3[i] = fiw3[i] - k2 * (r[i] - diw) * (vel[i] * tiw) * tiw

    # Force due to the desire to move as v0
    F1 = fi0

    # Other forces
    F2 = fij1 + fij2 + fij3 + fiw1 + fiw2 + fiw3

    return F1, F2, ang

##############################################################################################

def HSFM_system(t, X, N, n_groups, map_walls, num_walls, r, m, J, v0, group_membership):

    # Positions and velocities
    position = np.zeros((N, 2))
    vel = np.zeros((N, 2))
    for i in range(N):
        position[i, :] = [X[6 * i], X[6 * i + 1]]
        vel[i, :] = [X[6 * i + 3] * np.cos(X[6 * i + 2]), X[6 * i + 3] * np.sin(X[6 * i + 2])]

    e = waypoints.waypoint_update(position, 1.5)

    # Acting forces
    F0, Fe, ang = HSFM_forces(X, e, N, map_walls, num_walls, r, m, v0)
    FT = F0 + Fe

    # Magnitude of F0
    F_nV=(np.sqrt(np.sum(np.abs(F0)**2, 1)))

    #  desired theta
    thr = np.mod(ang, 2*np.pi).flatten()

    # actual theta
    th = np.mod(X.__getitem__(slice(2, None, 6)), 2*np.pi)

    # angle to rotate
    ang = np.unwrap(th - thr)
    # td = np.vstack((ang, ang+2*np.pi, ang-2*np.pi))
    # I = np.argmin(np.abs(td), 0)

    dX = np.zeros((6*N,1)).flatten()

    # center o of mass of each group
    if include_groups:
        ci = {}
        for k in range(len(n_groups)):
            ci[k] = np.array([0, 0])
    
        for i in range(N):
            ci[int(group_membership[i])] = ci[int(group_membership[i])] + position[i]
    
        for k in range(len(n_groups)):
            ci[k] = ci[k] / n_groups[k]

    for i in range(N):
        a = ang[i]#td[I[i], i]
        kl = 0.3
        kth = J[i] * kl * F_nV[i]
        kom = J[i] * (1+alpha) * np.sqrt(kl * F_nV[i] / alpha)
        
        if include_groups:
            p_i = ci[int(group_membership[i])]-position[i]

        dX[6*i] = X[6*i+3] * np.cos(X[6*i+2]) - X[6*i+4] * np.sin(X[6*i+2])
        dX[6*i+1] = X[6*i+3] * np.sin(X[6*i+2]) + X[6*i+4] * np.cos(X[6*i+2])
        dX[6*i+2] = X[6*i+5]

        if include_groups:
            # Here we substitute the step function in the definition of the group
            # cohesion forces with a sigmoid
            uf_group = k1g * (1+np.tanh(5*(np.abs(np.dot(p_i, [np.cos(X[6*i+2]), np.sin(X[6*i+2])])-d_f)-3))) * \
                       np.dot(p_i / np.linalg.norm(p_i), [np.cos(X[6*i+2]), np.sin(X[6*i+2])])
            uo_group = k2g * (1+np.tanh(5*(np.abs(np.dot(p_i, [-np.sin(X[6*i+2]), np.cos(X[6*i+2])])-d_o)-3))) * \
                       np.dot(p_i / np.linalg.norm(p_i), [-np.sin(X[6*i+2]), np.cos(X[6*i+2])])
    
            dX[6*i+3] = 1 / m[i] * (np.dot(FT[i], [np.cos(X[6*i+2]), np.sin(X[6*i+2])]) + uf_group)
            dX[6*i+4] = 1 / m[i] * (ko*np.dot(Fe[i], [-np.sin(X[6*i+2]), np.cos(X[6*i+2])]) - kd * X[6*i+4] + uo_group)
            dX[6*i+5] = 1 / J[i] * (-kth * a - kom * X[6*i+5])
        else:
            dX[6*i+3] = 1 / m[i] * (np.dot(FT[i], [np.cos(X[6*i+2]), np.sin(X[6*i+2])]) )
            dX[6*i+4] = 1 / m[i] * (ko*np.dot(Fe[i], [-np.sin(X[6*i+2]), np.cos(X[6*i+2])]) - kd * X[6*i+4] )
            dX[6*i+5] = 1 / J[i] * (-kth * a - kom * X[6*i+5])

    return dX

# ################################################################################


# System evolution
sol = ode(HSFM_system).set_integrator('dopri5')
t_start = 0.0
t_final = TF
delta_t = t_fine
# Number of time steps: 1 extra for initial condition
num_steps = int(np.floor((t_final - t_start)/delta_t) + 1)
sol.set_initial_value(X0, t_start)
sol.set_f_params(N, n_groups, map_walls, num_walls, r, m, J, v0, group_membership)

t = np.zeros((num_steps, 1))
X = np.zeros((num_steps, N*6))
t[0] = t_start
X[0] = X0
k = 1
while sol.successful() and k < num_steps:
    sol.integrate(sol.t + delta_t)
    t[k] = sol.t
    X[k] = sol.y
    print('time ', sol.t)
    k += 1


# Plotting example



colors = {0: "#0066CC", 1: "#006600"}
plt.figure()
# Plot of the walls
for i in range(num_walls):
    plt.plot(map_walls[2*i,:], map_walls[2*i+1,:],'k')
    plt.xlim([0,25])
    plt.ylim([0,25])
# Starting points
plt.plot(X[0].__getitem__(slice(0, None, 6)), X[0].__getitem__(slice(1, None, 6)), 'ro')
# Trajectories
for i in range(N):
    plt.plot(X[:, 6*i],X[:, 6*i+1], color=colors[int(i%2)])
plt.xlim([0,25])
plt.ylim([0,25])
# plt.axis('equal')
plt.savefig('trajectories.eps')
