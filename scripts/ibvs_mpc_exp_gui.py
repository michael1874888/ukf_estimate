import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from ukf_estimate.msg import output
from ukf_estimate.msg import Trajectory3D
from tf.transformations import euler_from_quaternion
from casadi import *
import Tkinter as tk

window = tk.Tk()
window.title('Tune Gain')
window.geometry('1000x600')
scale1 = tk.Scale(window, label='Q11 (x1)', from_=0, to=10, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=1, resolution=0.01)
scale2 = tk.Scale(window, label='Q22 (x2)', from_=0, to=10, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=1, resolution=0.01)
scale3 = tk.Scale(window, label='Q33 (x3)', from_=0, to=10, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=1, resolution=0.01)
scale4 = tk.Scale(window, label='Q44 (psi)', from_=0, to=10, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=1, resolution=0.01)
scale5 = tk.Scale(window, label='W11 (vcx)', from_=0, to=20, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=2, resolution=0.02)
scale6 = tk.Scale(window, label='W22 (vcy)', from_=0, to=20, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=2, resolution=0.02)
scale7 = tk.Scale(window, label='W33 (vcz)', from_=0, to=20, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=2, resolution=0.02)
scale8 = tk.Scale(window, label='W44 (wcy)', from_=0, to=10, orient=tk.HORIZONTAL, length=1000, showvalue=1, tickinterval=1, resolution=0.01)

# The time of the entire predict horizon is recommended to be greater than 1 second.
# In other words, N >= loop_rate
loop_rate = 20.
T = 1./loop_rate # Sampling Time
N = 30 # Predict horizon
image_width = 1280;
image_height = 720;
cx = 640
cy = 360
fx = 698.719
fy = 698.719
desired_distance = 0.6
desired_angle = 0
# Desired state value
sd_ = DM([(cx - cx)/fx, (cy - cy)/fy, 1./desired_distance, desired_angle])
des_pos = np.array([0.7, 0, 0.6, 0])
optimal_ibvs = False




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

sd1 = SX.sym('sd1')
sd2 = SX.sym('sd2')
sd3 = SX.sym('sd3')
sd4 = SX.sym('sd4')
sd = vertcat(sd1, sd2, sd3, sd4)

# Predict Model equations
x1dot = -x3*(u1-vq1) + x1*x3*(u3-vq3) - (1 + x1**2)*u4
x2dot = -x3*(u2-vq2) + x2*x3*(u3-vq3) - x1*x2*u4
x3dot = x3**2*(u3-vq3) - x1*x3*u4
x4dot = x3*(u1-vq1)
xdot = vertcat(x1dot, x2dot, x3dot, x4dot)




# Callback function
def callback(msg):
    global tar_vel, X_state_, optimal_ibvs
    tar_vel = np.array([msg.target_vel.x, msg.target_vel.y, msg.target_vel.z])
    X_state_ = DM([msg.feature_1.data, msg.feature_2.data, msg.feature_3.data, msg.feature_4.data])
    optimal_ibvs = msg.cmode.data
    #print('state',X_state_)
    #print('flag',optimal_ibvs)

host_mocap = PoseStamped()
def callback2(msg):
    global host_mocap
    host_mocap = msg
    #print(host_mocap)

def callback3(msg):
    global tar_acc
    tar_acc = np.array([msg.acc.x, msg.acc.y, msg.acc.z])
    print('acc',tar_acc)
    
def get_rotation(msg):
    orientation_list = [msg.x, msg.y, msg.z, msg.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print('roll',roll/pi*180,'pitch',pitch/pi*180,'yaw',yaw/pi*180)
    return roll, pitch, yaw
    
def position_control(des_pos, host_mocap, drone_yaw):
    
    errx = des_pos[0] - host_mocap.pose.position.x
    erry = des_pos[1] - host_mocap.pose.position.y
    errz = des_pos[2] - host_mocap.pose.position.z
    err_yaw = des_pos[3] - drone_yaw
    
    if err_yaw>pi:
        err_yaw = err_yaw - 2*pi
    elif err_yaw<-pi:
        err_yaw = err_yaw + 2*pi;
        
    pos_vel = TwistStamped()
    pos_vel.twist.linear.x = 1.0*errx
    pos_vel.twist.linear.y = 1.0*erry
    pos_vel.twist.linear.z = 1.0*errz
    pos_vel.twist.angular.z = 1.0*err_yaw
    
    return pos_vel

def update_function(Q, W):
    
    #print('Q',Q)
    #print('W',W)
    L = mtimes(mtimes((x-sd).T, Q), (x-sd)) + mtimes(mtimes((u-vq).T, W), (u-vq))

    # Formulate discrete time dynamics
    f = Function('f', [x, sd, u, vq], [xdot, L])
    X0 = SX.sym('X0', 4)
    Sd = SX.sym('Sd', 4)
    U = SX.sym('U', 4)
    Vq = SX.sym('Vq', 4)

    X = X0
    k1, C = f(X, Sd, U, Vq)
    X = X + k1*T
    F = Function('F', [X0, Sd, U, Vq], [X, C],['x0','xd','p','tv'],['xf','qf'])
    
    return F




rospy.init_node('mpc_ibvs', anonymous=True)
pub = rospy.Publisher('/drone1/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
estimate_sub = rospy.Subscriber("/estimate_data", output, callback, queue_size=1)
host_sub = rospy.Subscriber("/vrpn_client_node/RigidBody1/pose", PoseStamped, callback2, queue_size=1)
qp_sub = rospy.Subscriber("/target_qp", Trajectory3D, callback3, queue_size=1)
rate = rospy.Rate(loop_rate)

# Objective Parameters
Q = DM.eye(4)
W = DM.eye(4)
scale1.set(Q[0,0])
scale2.set(Q[1,1])
scale3.set(Q[2,2])
scale4.set(Q[3,3])
scale5.set(1.5)
scale6.set(1.2)
scale7.set(3)
scale8.set(0.5)
scale1.pack()
scale2.pack()
scale3.pack()
scale4.pack()
scale5.pack()
scale6.pack()
scale7.pack()
scale8.pack()


for i in range(0,100):
    print('spin')
    rate.sleep()    
    

# Formulate the NLP
#control_input = DM([0, 0, 0, 0])
tar_vel_cam = DM([0, 0, 0])
tar_vel_cam_pre = DM([0, 0, 0])
Xm = X_state_
cmd_vel = TwistStamped()


# Test the initial point
F = update_function(Q, W)
Fk = F(x0=X_state_,xd=sd_,p=[0, 0, 0, 0],tv=[0.1,0.1,0.1,0])
print(Fk['xf'])
print(Fk['qf'])




while not rospy.is_shutdown():
    
    #time1 = rospy.get_time()
    window.update()
    Q[0,0] = scale1.get()
    Q[1,1] = scale2.get()
    Q[2,2] = scale3.get()
    Q[3,3] = scale4.get()
    W[0,0] = scale5.get()
    W[1,1] = scale6.get()
    W[2,2] = scale7.get()
    W[3,3] = scale8.get()
    F = update_function(Q, W)
    
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
    tar_vel_cam_ = tar_vel_cam[:]
    #tar_acc_cam = mtimes(rot_g2c,tar_acc)
    tar_acc_cam = mtimes(rot_g2c,np.array([0,0,0]))
    

    # "Lift" initial conditions
    X_state = X_state_[:]                              # Ensure that lbw, ubw and w0 are the same
    X_state[3] = drone_yaw                             # Fixed heading on X-axis
    Xk = SX.sym('X0', 4)
    w += [Xk]
    lbw += list(X_state.full().flatten())
    ubw += list(X_state.full().flatten())
    w0 += list(X_state.full().flatten())
    
    epi = X_state - Xm
    Xd = sd_ - epi
    #print('sd',sd_)
    #print('xd',Xd)
    
    #time1 = rospy.get_time()

    for k in range(N):
        Uk = SX.sym('U_' + str(k), u.shape[0])
        w += [Uk]
        lbw += [-2, -2, -2, -0.6]
        ubw += [2, 2, 2, 0.6]
        w0 += [0, 0, 0, 0]

        # Integrate till the end of the interval
        Fk = F(x0=Xk, xd=Xd, p=Uk, tv=DM(vertcat(tar_vel_cam_,0)))
        Xk_end = Fk['xf']
        J=J+Fk['qf']
        
        # New NLP variable for state at end of interval
        Xk = SX.sym('X_' + str(k+1), 4)
        w   += [Xk]
        lbw += [(0 - cx)/fx, (0 - cy)/fy, 1./1.1, -pi/2]
        ubw += [(image_width - cx)/fx, (image_height - cy)/fy, 1/0.4, pi/2]
        w0  += [0, 0, 0, 0]
        
        tar_vel_cam_ = tar_vel_cam_ + tar_acc_cam*T
    
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
    
    # Predict the next state by model
    Fn = F(x0=X_state, xd=sd_, p=w_opt[4:8], tv=DM(vertcat(tar_vel_cam,0)))
    Xm = Fn['xf']
    
    # Applies the first control element of u, and update the states    
    # Calculate the command velocity in Global Frame
    uv_global = mtimes(rot_c2g, w_opt[4:7])
    uw_global = mtimes(rot_c2g, DM([0,w_opt[7],0]))
    
    fixed_vel = position_control(des_pos, host_mocap, drone_yaw)
    cmd_vel.twist.linear.x = uv_global[0]#fixed_vel.twist.linear.x#
    cmd_vel.twist.linear.y = uv_global[1]#fixed_vel.twist.linear.y#
    cmd_vel.twist.linear.z = uv_global[2]#fixed_vel.twist.linear.z#
    cmd_vel.twist.angular.z = uw_global[2]
    print(cmd_vel)
    
    if optimal_ibvs:
        pub.publish(cmd_vel)
        print('optimal_control')
        
    rate.sleep()
    
