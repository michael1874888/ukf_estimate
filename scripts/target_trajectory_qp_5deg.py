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

poly_degree = 5
window_length = 30
regulator_weight = 0.01

time = deque(maxlen = window_length)
target_position = deque(maxlen = window_length)
target_velocity = deque(maxlen = window_length)

target_data = output()
ti = 0.
callback_flag = False

rospy.init_node('target_trajectory_qp', anonymous=True)
rate = rospy.Rate(30)

current_time = rospy.get_time()
previous_time = rospy.get_time()
dt = 0.


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
        if callback_flag is True:
            time.append(ti)
            target_position.append(target_data.target_pose.y)
            target_velocity.append(target_data.target_vel.y)
            #print(time)
            #print(len(time))
            #print(target_position)

        if len(time) == window_length:

            ti = ti - time[0]
            
            t0 = time[0]
            for i in range(window_length):
                time[i] = time[i] - t0
            
            
            P_ = np.zeros(((poly_degree+1),(poly_degree+1)))
            q = np.zeros(((poly_degree+1), 1))
            
            for i in range(window_length):
            
    	        t = time[i]
                Ti0 = np.mat([[1],[t],[t**2],[t**3],[t**4],[t**5]])
                Tie0 = Ti0*Ti0.T
                Ti1 = np.mat([[0],[1],[2*t],[3*t**2],[4*t**3],[5*t**4]])
                Tie1 = Ti1*Ti1.T
                P_ = P_ + Tie0 + Tie1
                q = q + -2*target_position[i]*Ti0 + -2*target_velocity[i]*Ti1


            t_last = time[-1]
            t_init = time[0]
            
            Tir_last = np.mat([[0, 0,       0,       0,          0,          0],
                               [0, 0,       0,       0,          0,          0],
                               [0, 0,     4*t_last,  6*t_last**2,     8*t_last**3,    10*t_last**4],
                               [0, 0,  6*t_last**2, 12*t_last**3,    18*t_last**4,    24*t_last**5],
                               [0, 0,  8*t_last**3, 18*t_last**4, 144*t_last**5/5,    40*t_last**6],
                               [0, 0, 10*t_last**4, 24*t_last**5,    40*t_last**6, 400*t_last**7/7]])
                               
                               
            Tir_init = np.mat([[0, 0,       0,       0,          0,          0],
                               [0, 0,       0,       0,          0,          0],
                               [0, 0,     4*t_init,  6*t_init**2,     8*t_init**3,    10*t_init**4],
                               [0, 0,  6*t_init**2, 12*t_init**3,    18*t_init**4,    24*t_init**5],
                               [0, 0,  8*t_init**3, 18*t_init**4, 144*t_init**5/5,    40*t_init**6],
                               [0, 0, 10*t_init**4, 24*t_init**5,    40*t_init**6, 400*t_init**7/7]])
            '''
            Tir_last = np.mat([[0, 0, 0,        0,        0,        0],
                               [0, 0, 0,        0,        0,        0],
                               [0, 0, 0,        0,        0,        0],
                               [0, 0, 0,     36*t_last,  72*t_last**2, 120*t_last**3],
                               [0, 0, 0,  72*t_last**2, 192*t_last**3, 360*t_last**4],
                               [0, 0, 0, 120*t_last**3, 360*t_last**4, 720*t_last**5]])

                               
                               
            Tir_init = np.mat([[0, 0, 0,        0,        0,        0],
                               [0, 0, 0,        0,        0,        0],
                               [0, 0, 0,        0,        0,        0],
                               [0, 0, 0,     36*t_init,  72*t_init**2, 120*t_init**3],
                               [0, 0, 0,  72*t_init**2, 192*t_init**3, 360*t_init**4],
                               [0, 0, 0, 120*t_init**3, 360*t_init**4, 720*t_init**5]])
                               
            '''                   
            Tir = Tir_last - Tir_init
    
		    #Tir = np.array(Tir).astype(np.float64)
            
            P_ = P_ + window_length*regulator_weight*Tir
		    #extract 1/2 to be the standard form, so we need to multiply 2 to the original formula
            P = 2*P_
            #print(P)
            #print(q)


            coeff = cvxopt_solve_qp(P, q)

            target_pos_poly = coeff[0] + coeff[1]*t_last + coeff[2]*t_last**2 + coeff[3]*t_last**3 + coeff[4]*t_last**4 + coeff[5]*t_last**5
            target_vel_poly = coeff[1] + 2*coeff[2]*t_last + 3*coeff[3]*t_last**2 + 4*coeff[4]*t_last**3 + 5*coeff[5]*t_last**4
            target_acc_poly = 2*coeff[2] + 3*2*coeff[3]*t_last + 4*3*coeff[4]*t_last**2 + 5*4*coeff[5]*t_last**3
            
            target_qp.x = target_pos_poly
            target_qp.y = target_vel_poly
            target_qp.z = target_acc_poly
            #print(coeff)
            #print(time)

            pub.publish(target_qp)
            #time2 = rospy.get_time()
            #print((time2-time1))
        rate.sleep()




if __name__ == '__main__':
    quadratic_progamming()

