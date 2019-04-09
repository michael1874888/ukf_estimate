import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from ukf_estimate.msg import output
from tf.transformations import euler_from_quaternion
from casadi import *


loop_rate = 50.
T = 1./loop_rate # Sampling Time
N = 10 # Predict horizon
image_width = 640;
image_height = 480;
cx = 320
cy = 240
fx = 381.36
fy = 381.36
desired_distance = 7.0
desired_angle = pi/2

# Desired state value
sd = DM([(cx - cx)/fx, ((cy + 0.15*image_height) - cy)/fy, 1./desired_distance, desired_angle])
optimal_ibvs = False

# Callback function
def callback(msg):
    global tar_vel, X_state, optimal_ibvs
    #target_data = msg
    tar_vel = np.array([msg.target_vel.x, msg.target_vel.y, msg.target_vel.z])
    X_state = DM([msg.feature.x, msg.feature.y, msg.feature.z, msg.target_pose.x])
    #tar_vel = np.array([0,0,0])
    #X_state = DM([0,0,0.1,84/180*pi])
    optimal_ibvs = msg.target_pose.y
    #print('msg',msg)
    #print('vel',tar_vel)
    #print('state',X_state)
    #print('bool',optimal_ibvs)

drone_idx = 0
host_mocap = PoseStamped
def callback2(msg):
    global drone_idx, host_mocap
    
    while msg.name[drone_idx] != 'iris':        
        if drone_idx<(len(msg.name)-1):
            drone_idx += 1

    #print('name',msg.pose[drone_idx])
    host_mocap.pose = msg.pose[drone_idx];
    #host_mocap_vel = msg->twist[drone_idx];
    #car_pose.pose = msg->pose[car_idx];
    #car_vel = msg->twist[car_idx];


def get_rotation(msg):
    #orientation_q = msg.pose.orientation
    orientation_list = [msg.x, msg.y, msg.z, msg.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print('roll',roll/pi*180,'pitch',pitch/pi*180,'yaw',yaw/pi*180)
    return roll, pitch, yaw

# Declare model variables
x1 = SX.sym('x1')
x2 = SX.sym('x2')
x3 = SX.sym('x3')
x4 = SX.sym('x4')
x = vertcat(x1, x2, x3, x4)

u1 = SX.sym('u1')
u2 = SX.sym('u2')
u3 = SX.sym('u3')
u4 = SX.sym('u4')
u = vertcat(u1, u2, u3, u4)

vq1 = SX.sym('vq1')
vq2 = SX.sym('vq2')
vq3 = SX.sym('vq3')
vq4 = SX.sym('vq4')
vq = vertcat(vq1, vq2, vq3, vq4)

# Predict Model equations
x1dot = -x3*(u1-vq1) + x1*x3*(u3-vq3) - (1 + x1**2)*u4
x2dot = -x3*(u2-vq2) + x2*x3*(u3-vq3) - x1*x2*u4
x3dot = x3**2*(u3-vq3) - x1*x3*u4
x4dot = x3*(u1-vq1)
xdot = vertcat(x1dot, x2dot, x3dot, x4dot)

# Objective term
Q = DM.eye(4)
W = DM.eye(4)
Q[2,2] = 100
W[0,0] = 0.005
W[1,1] = 0.005
W[2,2] = 0.01
W[3,3] = 0.1
L = mtimes(mtimes((x-sd).T, Q), (x-sd)) + mtimes(mtimes((u-vq).T, W), (u-vq))

# Formulate discrete time dynamics
if True:
    f = Function('f', [x, u, vq], [xdot, L])
    X0 = SX.sym('X0', 4)
    U = SX.sym('U', 4)
    Vq = SX.sym('Vq', 4)
    X = X0
    Q = 0
    k1, k1_q = f(X, U, Vq)
    X = X + k1*T
    Q = Q + k1_q
    F = Function('F', [X0, U, Vq], [X, Q],['x0','p','tv'],['xf','qf'])
else:
    # Fixed step Runge-Kutta 4 integrator
    M = 4 # RK4 steps per interval
    DT = T/M
    f = Function('f', [x, u, vq], [xdot, L])
    X0 = SX.sym('X0', 4)
    U = SX.sym('U', 4)
    Vq = SX.sym('Vq', 4)
    X = X0
    Q = 0
    for j in range(M):
       k1, k1_q = f(X, U, Vq)
       k2, k2_q = f(X + DT/2 * k1, U, Vq)
       k3, k3_q = f(X + DT/2 * k2, U, Vq)
       k4, k4_q = f(X + DT * k3, U, Vq)
       X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
       Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
    F = Function('F', [X0, U, Vq], [X, Q],['x0','p','tv'],['xf','qf'])
'''
def state_update(cam_pose, tar_pos, u, vq):
    
    
    
    uv_global = mtimes(rot_c2g, u[0:3])
    uw_global = mtimes(rot_c2g, DM([0,u[3],0]))
    
    cam_pose_new = np.zeros(6)
    tar_pos_new = np.zeros(3)
    cam_pose_new[0:3] = (cam_pose[0:3] + uv_global*T).full().flatten()
    cam_pose_new[3:6] = (cam_pose[3:6] + uw_global*T).full().flatten()
    tar_pos_new = tar_pos + vq*T
    
    rqc_global = tar_pos_new - cam_pose_new[0:3]
    rqc_cam = mtimes(rot_g2c, rqc_global)
    
    X_state_new = DM(4,1)
    X_state_new[0] = rqc_cam[0]/rqc_cam[2]
    X_state_new[1] = rqc_cam[1]/rqc_cam[2]
    X_state_new[2] = 1./rqc_cam[2]
    X_state_new[3] = math.atan2((cam_pose_new[1]-tar_pos[1]), (cam_pose_new[0]-tar_pos[0])) - math.atan2(vq[1], vq[0])
    
    
    return cam_pose_new, tar_pos_new, X_state_new;
'''
rospy.init_node('mpc_ibvs', anonymous=True)
pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
estimate_sub = rospy.Subscriber("/estimate_data", output, callback, queue_size=1)
host_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback2, queue_size=1)   
rate = rospy.Rate(loop_rate)

for i in range(0,100):
    print('spin')
    rate.sleep()    
    
    


# Formulate the NLP
#control_input = DM([0, 0, 0, 0])
tar_vel_cam = DM([0, 0, 0])
tar_vel_cam_pre = DM([0, 0, 0])
cmd_vel = TwistStamped()

# Test the initial point
Fk = F(x0=X_state,p=[0, 0, 0, 0],tv=[1,1,1,0])
print(Fk['xf'])
print(Fk['qf'])

while not rospy.is_shutdown():
    
    #global rot_g2c, rot_c2g
    
    #time1 = rospy.get_time()
    
    # Start with an empty NLP
    w0=[]
    w=[]
    lbw = []
    ubw = []
    g=[]
    lbg = []
    ubg = []
    J = 0
    
    # Calculate the rotation matrix between Camera Frame and Global Frame
    drone_roll, drone_pitch, drone_yaw = get_rotation(host_mocap.pose.orientation)
    
    rotation_x = np.mat([[1,               0,                0],
                         [0, cos(drone_roll),  sin(drone_roll)],
                         [0,-sin(drone_roll),  cos(drone_roll)]])
                         
    rotation_y = np.mat([[cos(drone_pitch), 0, -sin(drone_pitch)],
                         [               0, 1,                 0],
                         [sin(drone_pitch), 0,  cos(drone_pitch)]])
                         
    rotation_z = np.mat([[ cos(drone_yaw), sin(drone_yaw), 0],
                         [-sin(drone_yaw), cos(drone_yaw), 0],
                         [0              ,              0, 1]])
    
    rotationB2C_y = np.mat([[cos(pi/2), 0, -sin(pi/2)],
                            [0,         1,          0],
                            [sin(pi/2), 0,  cos(pi/2)]])
    
    rotationB2C_z = np.mat([[ cos(-pi/2), sin(-pi/2), 0],
                            [-sin(-pi/2), cos(-pi/2), 0],
                            [0          ,          0, 1]])
    
    rot_g2c = rotationB2C_z*rotationB2C_y*rotation_z
    rot_c2g = rot_g2c.T
    
    # Calculate the target velocity in Camera Frame
    tar_vel_cam = mtimes(rot_g2c,tar_vel)
    

    # "Lift" initial conditions
    Xk = SX.sym('X0', 4)
    w += [Xk]
    lbw += list(X_state.full().flatten())
    ubw += list(X_state.full().flatten())
    w0 += list(X_state.full().flatten())
    #time1 = rospy.get_time()

    for k in range(N):
        Uk = SX.sym('U_' + str(k), u.shape[0])
        w += [Uk]
        lbw += [-8, -6, -6, -0.6]
        ubw += [6, 6, 6, 0.6]
        w0 += [0,0,0,0]

        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=Uk, tv=DM(vertcat(tar_vel_cam,0)))
        Xk_end = Fk['xf']
        J=J+Fk['qf']
        
        # New NLP variable for state at end of interval
        Xk = SX.sym('X_' + str(k+1), 4)
        w   += [Xk]
        lbw += [(0 - cx)/fx, (0 - cy)/fy, 1./15, 0]
        ubw += [(image_width - cx)/fx, (image_height - cy)/fy, 1, pi]
        w0  += [0, 0, 0, 0]
    
        # Add inequality constraint
        g   += [Xk_end-Xk]
        lbg += [0, 0, 0, 0]
        ubg += [0, 0, 0, 0]

    # Create an NLP solver
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    opts={}
    opts["verbose_init"] = False
    opts["verbose"] = False
    opts["print_time"] = False
    opts["ipopt.print_level"] = 0
    solver = nlpsol('solver', 'ipopt', prob, opts);
    
    # Solve the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x']
    
    # Applies the first control element of u, and update the states
    #control_input = control_input + w_opt[4:8]
    
    # Calculate the command velocity in Global Frame
    uv_global = mtimes(rot_c2g, w_opt[4:7])
    uw_global = mtimes(rot_c2g, DM([0,w_opt[7],0]))
    
    
    cmd_vel.twist.linear.x = uv_global[0]
    cmd_vel.twist.linear.y = uv_global[1]
    cmd_vel.twist.linear.z = uv_global[2]
    cmd_vel.twist.angular.z = uw_global[2]
    print(cmd_vel)
    
    if optimal_ibvs:
        pub.publish(cmd_vel)
        print('optimal_control')
        
    rate.sleep()
'''    
    # Applies the first control element of u, and update the states
    cam_pose, tar_pos, X_state = state_update(cam_pose, tar_pos, w_opt[x.shape[0]:(x.shape[0]+u.shape[0])], tar_vel)
    X_states += [X_state.full()]
    input_opt += [w_opt[x.shape[0]:(x.shape[0]+u.shape[0])].full()]
    cam_measures += [cam_pose]
    tar_measures += [tar_pos]
    tar_vel_measures += [tar_vel]
    time2 = rospy.get_time()
    print((time2-time1))
    
    # Update Target Velocity
    if mpc_iter < 80:
        tar_vel += 0.05
    
    mpc_iter += 1

# Plot the solution
u_opt = []
v_opt = []
d_opt = []
psi_opt = []

vcx_opt = []
vcy_opt = []
vcz_opt = []
wcy_opt = []

vec_scale = 3
cam_vec_x = []
cam_vec_y = []
tar_vec_x = []
tar_vec_y = []
tar_vec_z = []

cam_x = []
cam_y = []
cam_z = []
tar_x = []
tar_y = []
tar_z = []

for r in X_states:
    u_opt += [(r[0]*fx + cx)]
    v_opt += [(r[1]*fy + cy)]
    d_opt += [(1/r[2])]
    psi_opt += [(r[3]/pi*180)]

for r in input_opt:
    vcx_opt += [r[0]]
    vcy_opt += [r[1]]
    vcz_opt += [r[2]]
    wcy_opt += [r[3]]
    
for r in cam_measures:
    cam_vec_x += [vec_scale*cos(r[5])]
    cam_vec_y += [vec_scale*sin(r[5])]
    cam_x += [r[0]]
    cam_y += [r[1]]
    cam_z += [r[2]]
    
for r in tar_measures:
    tar_x += [r[0]]
    tar_y += [r[1]]
    tar_z += [r[2]]
    
for i in range(1,len(tar_measures)):
    tar_vec_x += [tar_x[i] - tar_x[i-1]]
    tar_vec_y += [tar_y[i] - tar_y[i-1]]
    tar_vec_z += [tar_z[i] - tar_z[i-1]]
    
tgrid = [T*k for k in range(mpciteratios+1)]

print('u', u_opt)
print('v', v_opt)
print('depth', d_opt)
print('psi', psi_opt)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

plt.figure(1)
plt.clf()
plt.plot(tgrid, u_opt, '--')
plt.plot(tgrid, v_opt, '-')
plt.title('U/V w.r.t. Time')
plt.xlabel('t')
plt.ylabel('pixel')
plt.legend(['u','v'])
plt.grid()

plt.figure(2)
plt.clf()
plt.plot(u_opt, v_opt, '-')
plt.title('U/V Closed Loop Trajectory', y=1.08)
plt.xlabel('U')
plt.ylabel('V')
plt.xlim((0, 640))
plt.ylim((0, 480))
plt.gca().xaxis.set_ticks_position('top')
plt.gca().invert_yaxis()
plt.grid()

plt.figure(3)
plt.clf()
plt.plot(tgrid[0:-1], vcx_opt, '-')
plt.plot(tgrid[0:-1], vcy_opt, '--')
plt.plot(tgrid[0:-1], vcz_opt, '-.')
plt.plot(tgrid[0:-1], wcy_opt, '-o')
plt.title('Control Input')
plt.xlabel('t')
plt.ylabel('V')
plt.legend(['vcx','vcy','vcz','wcy'])
plt.grid()

plt.figure(4)
plt.clf()
ax = plt.gca(projection='3d')
ax.quiver(cam_x, cam_y, cam_z, cam_vec_x, cam_vec_y, 0, normalize=True, color='r')
ax.quiver(tar_x[0:-1], tar_y[0:-1], tar_z[0:-1], tar_vec_x, tar_vec_y, tar_vec_z, normalize=True, color='b')
plt.title('Trajectory')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(min(min(cam_x),min(tar_x))-1,max(max(cam_x),max(tar_x))+1)
ax.set_ylim(min(min(cam_y),min(tar_y))-1,max(max(cam_y),max(tar_y))+1)
ax.set_zlim(min(min(cam_z),min(tar_z))-1,max(max(cam_z),max(tar_z))+1)
plt.grid()

plt.figure(5)
plt.clf()
plt.plot(tgrid, d_opt, '--')
plt.plot(tgrid, psi_opt, '-')
plt.title('U/V w.r.t. Time')
plt.xlabel('t')
plt.ylabel('Depth and psi')
plt.legend(['d','psi'])
plt.grid()


plt.show()
'''

