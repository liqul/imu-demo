import math
import os
import sys
import numpy as np


def qprod(a, b):
    c = [0,0,0,0]
    c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
    c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
    c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
    c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
    return c

def q2e(q):
    yaw = math.atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1)
    theta = -math.asin(2*q[1]*q[3] + 2*q[0]*q[2])
    psi = math.atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1)
    return (phi, theta, psi)

# data file format in
# timestamp, gx, gy, gz
lines = open(sys.argv[1]).readlines()

cur_q = [1, 0, 0, 0]

for line in lines:
    data = [float(x) for x in line.strip('\n').split(',')]

    # derive the quaternion representation
    q_rate = qprod(init_q, [0, data[1], data[2], data[3]])
    cur_q[0] += q_rate[0] * 0.01
    cur_q[1] += q_rate[1] * 0.01
    cur_q[2] += q_rate[2] * 0.01
    cur_q[3] += q_rate[3] * 0.01
    
    print q2e(cur_q)



