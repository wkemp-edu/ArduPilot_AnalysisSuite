#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 19 15:43:24 2022

@author: williamkemp
"""
import numpy as np

class U7V2_280KV:
    
    def __init__(self):
        self.KV = 280 # From motor specifications

    def efficiency(self, n, current):
        # Motor efficiency of U7V2 with zubax controller
        # Inputs:
        # 1. Rotations per second (n)
        # 2. Current draw (Measured by the ESC)
        # Outputs:
        # 1. Motor efficiency

        # Parameters from fitting
        # R^2 = 0.98939 
        i00 = -0.04156;
        i01 = 0.001057;
        i02 = -2.905e-7;
        
        Kv = 280;
        Kq = 304.5;
        tau = -0.000171;
        R = 0.01586;

        # Modifying inputs:
        omega = 2 * np.pi * n
        i0 = i00 + i01*omega + i02*omega**2
        
        efficiency = (1 - (i0/current)) * (Kv/Kq) * (1 + tau*omega + current*R*Kv*omega**-1)**-1;
        return efficiency

    def motor_current(self, torque):
        # Finding the current through ESC from the torque: based on fit of WT data
        Q = torque
        i = 17.98 * Q**2 + 8.493 * Q**1 + 0.363
        return i