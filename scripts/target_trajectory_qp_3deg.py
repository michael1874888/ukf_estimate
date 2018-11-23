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
from geometry_msgs.msg import Point

poly_degree = 3
window_length = 60
regulator_weight = 1

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

    global ti, time, P_, qx, qy, qz
    
    rospy.Subscriber("/estimate_data", output, callback)   
    pub_pos = rospy.Publisher('target_qp_pos', Point, queue_size=1)
    pub_vel = rospy.Publisher('target_qp_vel', Point, queue_size=1)
    pub_acc = rospy.Publisher('target_qp_acc', Point, queue_size=1)

    target_qp_pos = Point()
    target_qp_vel = Point()
    target_qp_acc = Point()
    
    while not rospy.is_shutdown():
        #time1 = rospy.get_time()
        if callback_flag is True:
            time.append(ti)
            target_position.append(target_data.target_pose)
            target_velocity.append(target_data.target_vel)
            #print(time)
            #print(len(time))
            #print(target_position)

        if len(time) == window_length:

            ti = ti - time[0]
            
            t0 = time[0]
            for i in range(window_length):
                time[i] = time[i] - t0
            
            
            P_ = np.zeros(((poly_degree+1),(poly_degree+1)))
            qx = np.zeros(((poly_degree+1), 1))
            qy = np.zeros(((poly_degree+1), 1))
            qz = np.zeros(((poly_degree+1), 1))
            
            for i in range(window_length):
            
    	        t = time[i]
                Ti0 = np.mat([[1],[t],[t**2],[t**3]])
                Tie0 = Ti0*Ti0.T
                Ti1 = np.mat([[0],[1],[2*t],[3*t**2]])
                Tie1 = Ti1*Ti1.T
                P_ = P_ + Tie0 + Tie1
                qx = qx + -2*target_position[i].x*Ti0 + -2*target_velocity[i].x*Ti1
                qy = qy + -2*target_position[i].y*Ti0 + -2*target_velocity[i].y*Ti1
                qz = qz + -2*target_position[i].z*Ti0 + -2*target_velocity[i].z*Ti1


            t_last = time[-1]
            t_init = time[0]
            
            #Tir for acceleration regulator
            Tir_last = np.mat([[0, 0,           0,            0],
                               [0, 0,           0,            0],
                               [0, 0,    4*t_last,  6*t_last**2],
                               [0, 0, 6*t_last**2, 12*t_last**3]])
                               
                               
            Tir_init = np.mat([[0, 0,           0,            0],
                               [0, 0,           0,            0],
                               [0, 0,    4*t_init,  6*t_init**2],
                               [0, 0, 6*t_init**2, 12*t_init**3]])
                  
            Tir = Tir_last - Tir_init
    
		    #Tir = np.array(Tir).astype(np.float64)
            
            P_ = P_ + window_length*regulator_weight*Tir
		    #extract 1/2 to be the standard form, so we need to multiply 2 to the original formula
            P = 2*P_
            #print(P)
            #print(q)


            coeff_x = cvxopt_solve_qp(P, qx)
            coeff_y = cvxopt_solve_qp(P, qy)
            coeff_z = cvxopt_solve_qp(P, qz)

            target_pos_poly_x = coeff_x[0] + coeff_x[1]*t_last + coeff_x[2]*t_last**2 + coeff_x[3]*t_last**3
            target_pos_poly_y = coeff_y[0] + coeff_y[1]*t_last + coeff_y[2]*t_last**2 + coeff_y[3]*t_last**3
            target_pos_poly_z = coeff_z[0] + coeff_z[1]*t_last + coeff_z[2]*t_last**2 + coeff_z[3]*t_last**3
            target_vel_poly_x = coeff_x[1] + 2*coeff_x[2]*t_last + 3*coeff_x[3]*t_last**2
            target_vel_poly_y = coeff_y[1] + 2*coeff_y[2]*t_last + 3*coeff_y[3]*t_last**2
            target_vel_poly_z = coeff_z[1] + 2*coeff_z[2]*t_last + 3*coeff_z[3]*t_last**2
            target_acc_poly_x = 2*coeff_x[2] + 3*2*coeff_x[3]*t_last
            target_acc_poly_y = 2*coeff_y[2] + 3*2*coeff_y[3]*t_last
            target_acc_poly_z = 2*coeff_z[2] + 3*2*coeff_z[3]*t_last
            
            target_qp_pos.x = target_pos_poly_x
            target_qp_pos.y = target_pos_poly_y
            target_qp_pos.z = target_pos_poly_z
            target_qp_vel.x = target_vel_poly_x
            target_qp_vel.y = target_vel_poly_y
            target_qp_vel.z = target_vel_poly_z
            target_qp_acc.x = target_acc_poly_x
            target_qp_acc.y = target_acc_poly_y
            target_qp_acc.z = target_acc_poly_z
            #print(coeff)
            #print(time)

            pub_pos.publish(target_qp_pos)
            pub_vel.publish(target_qp_vel)
            pub_acc.publish(target_qp_acc)
            #time2 = rospy.get_time()
            #print((time2-time1))
        rate.sleep()




if __name__ == '__main__':
    quadratic_progamming()

