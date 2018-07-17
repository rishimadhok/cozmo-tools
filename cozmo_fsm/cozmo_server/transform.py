"""
Transformation matrices for kinematics calculations.
"""

import numpy as np
from numpy import arctan2, array
from math import sin, cos, tan, pi, sqrt

def rotationMatrixToEulerAngles(R) :
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = arctan2(R[2,1] , R[2,2])
        y = arctan2(-R[2,0], sy)
        z = arctan2(R[1,0], R[0,0])
    else :
        x = arctan2(-R[1,2], R[1,1])
        y = arctan2(-R[2,0], sy)
        z = 0

    return array([x, y, z])

def point(x=0,y=0,z=0):
    return np.array([ [x], [y], [z], [1.] ])

def norm(pt):
    return pt[0][0:3].norm()

def aboutX(theta):
    c = cos(theta)
    s = sin(theta)
    return np.array([
        [ 1,  0,  0, 0],
        [ 0,  c, -s, 0],
        [ 0,  s,  c, 0],
        [ 0,  0,  0, 1]])

def aboutY(theta):
    c = cos(theta)
    s = sin(theta)
    return np.array([
        [ c,  0,  s, 0],
        [ 0,  1,  0, 0],
        [-s,  0,  c, 0],
        [ 0,  0,  0, 1]])

def aboutZ(theta):
    c = cos(theta)
    s = sin(theta)
    return np.array([
        [ c, -s,  0, 0],
        [ s,  c,  0, 0],
        [ 0,  0,  1, 0],
        [ 0,  0,  0, 1.]])

def translate(x,y,z=0):
    return np.array([
        [ 1, 0, 0, x],
        [ 0, 1, 0, y],
        [ 0, 0, 1, z],
        [ 0, 0, 0, 1.]])

def normalize(v):
    s = v[3,0]
    if s == 0:
        return v
    else:
        return v/s

def identity():
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1.]])

def dh_matrix(d,theta,r,alpha):
    """Denavit-Hartenberg transformation from joint i to joint i+1."""
    return aboutX(alpha).dot(translate(r,0,d).dot(aboutZ(theta)))

def translation(t):
    return np.array([ [t[0,3]], [t[1,3]], [t[2,3]], [t[3,3]] ])

def wrap_angle(angle_rads):
    """Keep angle between -pi and pi."""
    if angle_rads <= -pi:
        return 2*pi + angle_rads
    elif angle_rads > pi:
        return angle_rads - 2*pi
    else:
        return angle_rads

def wrap_selected_angles(angle_rads, index):
    """Keep angle between -pi and pi for list"""
    for i in index:
       angle_rads[i] =  wrap_angle(angle_rads[i])
    return angle_rads

def tprint(t):
    number_format = "%7.3f"
    def tprint_vector(t):
        for i in range(t.shape[0]):
            if i == 0:
                print('[ ',end='')
            else:
                print('  ',end='')
            print(number_format % t[i],end='')
            if i+1 == t.shape[0]:
                print(' ]')
            else:
                print()
    def tprint_matrix(t):
        for i in range(t.shape[0]):
            if i == 0:
                print('[ ',end='')
            else:
                print('  ',end='')
            for j in range(t.shape[1]):
                if j>0: print('  ',end='')
                print(number_format % t[i][j], end='')
            if i+1 == t.shape[0]:
                print(' ]')
            else:
                print()
    if isinstance(t, np.ndarray) and t.ndim == 1:
        tprint_vector(t)
    elif isinstance(t, np.ndarray) and t.ndim == 2:
        tprint_matrix(t)
    elif isinstance(t, (int,float)):
        print(number_format % t)
    else:
        print(t)

def quat2rot(q0,q1,q2,q3):
    # formula from http://stackoverflow.com/questions/7938373/from-quaternions-to-opengl-rotations
    q0_sq = q0*q0; q1_sq = q1*q1; q2_sq = q2*q2; q3_sq = q3*q3
    t_q0q1 = 2. * q0 * q1
    t_q0q2 = 2. * q0 * q2
    t_q0q3 = 2. * q0 * q3
    t_q1q2 = 2. * q1 * q2
    t_q1q3 = 2. * q1 * q3
    t_q2q3 = 2. * q2 * q3
    return np.array([
        [ q0_sq+q1_sq-q2_sq-q3_sq, t_q1q2-t_q0q3,           t_q1q3+t_q0q2,           0. ],
        [ t_q1q2+t_q0q3,           q0_sq-q1_sq+q2_sq-q3_sq, t_q2q3-t_q0q1,           0. ],
        [ t_q1q3-t_q0q2,           t_q2q3+t_q0q1,           q0_sq-q1_sq-q2_sq+q3_sq, 0. ],
        [             0.,                     0.,                      0.,           1.  ]])


#---------------- General Geometric Calculations ----------------

def project_to_line(x0,y0,theta0,x1,y1):
    """Returns the projection of the point (x1,y1) onto the
    line through (x0,y0) with orientation theta0."""
    bigvalue = 1e6
    m0 = max(-bigvalue, min(bigvalue, tan(theta0)))
    if abs(m0) < 1/bigvalue:
        return (x1,y0)
    m1 = -1 / m0
    b0 = y0 - m0*x0
    b1 = y1 - m1*x1
    x2 = (b0-b1) / (m1-m0)
    y2 = m0 * x2 + b0
    return (x2,y2)
