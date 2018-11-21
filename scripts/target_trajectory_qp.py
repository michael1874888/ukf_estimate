import rospy
from std_msgs.msg import Float64
from ukf_estimate.msg import output
import cvxopt
from cvxopt import matrix
import numpy as np
import sympy as sym
from collections import deque
import matplotlib.pyplot as plt
import time
from geometry_msgs.msg import Vector3

poly_degree = 3
window_length = 10
regulator_weight = 0.000
#regulator_diff = 2
t = sym.symbols("t")
time = deque(maxlen = window_length)
target_position = deque(maxlen = window_length)
target_velocity = deque(maxlen = window_length)
#Ti0 = deque(maxlen = window_length)
#Tie0 = deque(maxlen = window_length)
#Ti1 = deque(maxlen = window_length)
#Tie1 = deque(maxlen = window_length)
Ti_sym = sym.zeros((poly_degree+1),1)
target_data = output()
ti = 0.
callback_flag = False

rospy.init_node('target_trajectory_qp', anonymous=True)
rate = rospy.Rate(30)

current_time = rospy.get_time()
previous_time = rospy.get_time()
dt = 0.

#Solve QP problem
def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None):
    P = .5 * (P + P.T)  # make sure P is symmetric
    args = [matrix(P), matrix(q)]
    if G is not None:
        args.extend([matrix(G), matrix(h)])
        if A is not None:
            args.extend([matrix(A), matrix(b)])
    sol = cvxopt.solvers.qp(*args)
    if 'optimal' not in sol['status']:
        return None
    return np.array(sol['x']).reshape((P.shape[1],))

#Estimation data of the target from UKF
def callback(msg):
    global callback_flag, target_data, current_time, previous_time, ti
    callback_flag = True
    target_data = msg
    current_time = rospy.get_time()
    dt = current_time - previous_time
    #print(dt)
    previous_time = current_time
    ti = ti + dt
    #rospy.Rate(30).sleep()
    
    


def quadratic_progamming():

    global ti, time, P_, q
    
    rospy.Subscriber("/estimate_data", output, callback)   
    pub = rospy.Publisher('qp_pos', Vector3, queue_size=1)

    target_qp = Vector3()
    while not rospy.is_shutdown():
        #time1 = rospy.get_time()
        #Start if the target data is subscribed
        if callback_flag is True:
            time.append(ti)
            target_position.append(target_data.target_pose.y)
            target_velocity.append(target_data.target_vel.y)
            #print(time)
            #print(len(time))
            #print(target_position)

        if len(time) == window_length:

            #Shift the time stamp
            ti = ti - time[0]
            
            t0 = time[0]
            for i in range(window_length):
                time[i] = time[i] - t0
            
            #print(time)
            #Declare Ti with symbol "t"
            for i in range((poly_degree+1)):
			    Ti_sym[i] = t**i
            
            Ti_sym_diff = Ti_sym.diff(t)
            
            P_ = sym.zeros((poly_degree+1),(poly_degree+1))
            q = sym.zeros((poly_degree+1), 1)
            
            #Calculate Ti and Te to determine P and q for CVXOPT
            for i in range(window_length):
                #print(i)
                #print(time[i])
    	        Ti0 = Ti_sym.subs(t, time[i])
                #Ti0.append(Ti0_)
                Tie0 = Ti0*Ti0.T
                Ti1 = Ti_sym_diff.subs(t, time[i])
                #Ti1.append(Ti1_)
                Tie1 = Ti1*Ti1.T
                P_ = P_ + Tie0 + Tie1
                q = q + -2*target_position[i]*Ti0 + -2*target_velocity[i]*Ti1

            #Calculate Tir to determine P for CVXOPT
            #Tir for acceleration regulator    
            Ti_sym_diff2 = Ti_sym_diff.diff(t)
            Tir_ = Ti_sym_diff2*Ti_sym_diff2.T
            Tir_ = Tir_.integrate(t)
            Tir = Tir_.subs(t,time[-1]) - Tir_.subs(t,time[0])
		    #Tir = np.array(Tir).astype(np.float64)
            
            P_ = P_ + window_length*regulator_weight*Tir
		    #extract 1/2 to be the standard form, so we need to multiply 2 to the original formula
            P = 2*P_
            P = np.array(P).astype(np.float64)
            q = np.array(q).astype(np.float64)
            #print(P)
            #print(q)


            coeff = cvxopt_solve_qp(P, q)
            target_pos_poly = sym.Matrix(coeff).dot(Ti_sym)
                
            target_vel_poly = target_pos_poly.diff(t)
            target_acc_poly = target_vel_poly.diff(t)
            
            target_qp.x = target_pos_poly.subs(t, time[-1])
            target_qp.y = target_vel_poly.subs(t, time[-1])
            target_qp.z = target_acc_poly.subs(t, time[-1])
            #print(coeff)
            #print(time)

            pub.publish(target_qp)
            #time2 = rospy.get_time()
            #print((time2-time1))
        rate.sleep()




if __name__ == '__main__':
    quadratic_progamming()

