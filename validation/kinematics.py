#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 24 09:31:08 2019

@author: tim
@brief: validate kinematic transformations
"""

import numpy as np
from math import atan2, acos, cos, sin, sqrt, pi, asin

def quat2longlat(beta):
    phi = asin(2*(beta[1]*beta[3]-beta[0]*beta[2]))
    gamma = atan2(-2*(beta[1]*beta[2]-beta[0]*beta[3]),1-2*beta[1]**2-2*beta[3]**2)
    return (gamma,phi)

def longlat2quat(gamma,phi):
    # trace of DCM
    zeta = cos(phi)*cos(gamma)+cos(gamma)+cos(phi)
    b1 = sin(phi)*sin(gamma)/(2*sqrt(zeta+1))
    b2 = -sin(phi)*(1+cos(gamma))/(2*sqrt(zeta+1))
    b3 = (sin(gamma)+cos(phi)*sin(gamma))/(2*sqrt(zeta+1))
    b0 = 0.5*sqrt(zeta+1)
    return np.array([b0,b1,b2,b3])

def skew(x):
    rout = np.zeros((3,3))
    rout[0,1] = -x[2]
    rout[0,2] = x[1]
    rout[1,2] = -x[0]
    rout[1,0] = x[2]
    rout[2,0] = -x[1]
    rout[2,1] = x[0]
    return rout
def C2(x):
    C = np.array([[cos(x),0,-sin(x)],\
                   [0,1,0],\
                   [sin(x),0,cos(x)]])
    return C
def C3(x):
    C = np.array([[cos(x),sin(x),0],\
                   [-sin(x),cos(x),0],\
                   [0,0,1]])
    return C

sam = 1000

for k in range(sam):
    # sample
    gamma = np.random.uniform(-pi,pi)
    phi = np.random.uniform(-pi/2,pi/2)
    # quaternion
    qv = longlat2quat(gamma,phi)
    # convert back
    gammach,phich = quat2longlat(qv)
    # error
    err = sqrt((gamma-gammach)**2+(phi-phich)**2)
    if abs(err) > 1e-10:
        print(k+1,gamma,phi,err)

# DCM from quat
Cquat = np.eye(3)-2*qv[0]*skew(qv[1:])+2*np.dot(skew(qv[1:]),skew(qv[1:]))
# DCM from lon lat
Cll = np.dot(C2(-phi),C3(gamma))