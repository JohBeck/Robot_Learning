# CTL is the name of the controller.
# Q_HISTORY is a matrix containing all the past position of the robot. Each row of this matrix is [q_1, ... q_i], where
# i is the number of the joints.
# Q and QD are the current position and velocity, respectively.
# Q_DES, QD_DES, QDD_DES are the desired position, velocity and acceleration, respectively.
# GRAVITY is the gravity vector g(q).
# CORIOLIS is the Coriolis force vector c(q, qd).
# M is the mass matrix M(q).

import numpy as np
from math import pi


def my_ctl(ctl, q, qd, q_des, qd_des, qdd_des, q_hist, q_deshist, gravity, coriolis, M):
    #  1.2 c) use K_P = 60, K_D = 10, K_L = 0.1 for the first joint
    #  K_P = 30, K_L = 0.1, q_des = [-pi/2,0]
    K_P = np.diag([60, 30])
    K_D = np.diag([10, 6])
    K_I = [0.1, 0.1]
    if ctl == 'P':
        # u_t = K_P(q_des-q)
        u = K_P.dot((np.mat(q_des).T - np.mat(q).T))
    elif ctl == 'PD':
        # u_t = K_P(q_des-q_t)+K_D(qd_des-qd)
        u = np.dot(K_P, (np.mat(q_des).T - np.mat(q).T)) + np.dot(K_D, (np.mat(qd_des).T - np.mat(qd).T))
    elif ctl == 'PID':
        # u_t = K_P(q_des-q)+K_D(qd_des-qd)+K_I*all-time steps
        u = K_P * (np.mat(q_des).T - np.mat(q).T) + K_D * (np.mat(qd_des).T - np.mat(qd).T) + K_I * np.trapz(np.mat(q_deshist-q_hist).T)
    elif ctl == 'PD_Grav':
        # u_t = K_P(q_des-q)+K_D(qd_des-qd)+g(q)
        u = K_P * (np.mat(q_des).T - np.mat(q).T) + K_D * (np.mat(qd_des).T - np.mat(qd).T) + np.mat(gravity).T
    elif ctl == 'ModelBased':
        # u = M(q)*qdd_ref+c(qd,q) + g(q)
        # qdd_ref = qdd_des+K_D(qd_des-qd)+K_P(q_des-q)
        qdd_ref = np.mat(qdd_des).T+K_D*(np.mat(qd_des).T-np.mat(qd).T) + K_P * (np.mat(q_des).T - np.mat(q).T)
        u = M * qdd_ref + np.mat(coriolis).T + np.mat(gravity).T
    return u
