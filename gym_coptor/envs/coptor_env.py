# -*- coding: utf-8 -*-
"""
Created on Sat Jul 31 18:05:07 2021

@author: Sai Sathvik
"""

import numpy as np
import matplotlib.pyplot as plt
import math
import random
import gym
from gym import error, spaces, utils
from gym.utils import seeding

class omniCopter(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self):
        self.running = True
        self.m = 1
        self.Ixx = 0.2
        self.l = 0.2
        
    def calcDerivatives(self,x,act,xd):
        # Extract the actions
        Tr = act[0]
        phird = act[1]
        Tl = act[2]
        phild = act[3]
        # Calculate the rotor forces in earth axes
        # Create the tilting rotor direction cosines
        phir = x[6]
        phil = x[7]
        Cpr_b = np.array([[1,0,0],
                          [0,math.cos(phir),-math.sin(phir)],
                          [0,math.sin(phir),math.cos(phir)]])
        Cpl_b = np.array([[1,0,0],
                          [0,math.cos(phil),-math.sin(phil)],
                          [0,math.sin(phil),math.cos(phil)]])
        Tvr = np.array([[0.0],[0.0],-Tr],dtype=object)
        Tvl = np.array([[0.0],[0.0],-Tl],dtype=object)
        Fr_b = Cpr_b @ Tvr
        Fl_b = Cpl_b @ Tvl
        # Now the body to NED axes
        phi = x[4]
        Cb_e = np.array([[1,0,0],
                          [0,math.cos(phi),-math.sin(phi)],
                          [0,math.sin(phi),math.cos(phi)]])
        # Then,
        Fr_e = Cb_e @ Fr_b
        Fl_e = Cb_e @ Fl_b
        # Total forces acting on the body are then,
        g = 10
        F = Fr_e + Fl_e + Cb_e @ np.array([[0],[0],[self.m * g]],dtype=object)
        # Now the moments. First transgform the moment arms into NED axes
        r_cg_pr_e = Cb_e @ np.array([[0],[self.l],[0]],dtype=object)
        r_cg_pl_e = Cb_e @ np.array([[0],[-self.l],[0]],dtype=object)
        # Now calculate the torque vector
        Tq = np.cross(np.transpose(r_cg_pr_e),np.transpose(Fr_e)) \
           + np.cross(np.transpose(r_cg_pl_e),np.transpose(Fl_e)) 
        #
        # With the forces and moments found, we can compute the linear and 
        # angular accelerations.
        ydd = F[1][0] / self.m
        zdd = F[2][0] / self.m
        phidd = Tq[0][1] / self.Ixx
        # Return the derivative vectors
        xd[0] = x[1]
        xd[1] = ydd
        xd[2] = x[3]
        xd[3] = zdd
        xd[4] = x[5]
        xd[5] = phidd
        xd[6] = phird
        xd[7] = phild
        return xd
        
    def runningStatus(self,s):
        self.running = s
        return self.running