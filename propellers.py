#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 28 03:11:21 2022

@author: williamkemp
"""

# Updated on October 20th 2022
# Based on curve fits in Createv/Analysis/ file system
# Based on data from: (Yoke drag from 2022-04-18, 2022-04-13)
    # 20 x 8
    # * 2022-05-12
    # * 2022-04-19
    # * 2022-04-14
    # * 2022-04-13
    
    # 18.5 x 12
    # * 2022-05-07
    # * 2022-04-19
    # * 2021-09-10

class aeronaut20x8:
    
    def __init__(self):
        self.diameter = 0.512 # meters

    def efficiency(self, J):
        # Overall efficiency for a 20x8 system, from CP and CT fits
        eff = J * (self.thrust_coeff(J)/self.power_coeff(J))
        return eff
    
    def thrust_coeff(self, J):
        # Overall thrust coefficient for a 20x8 system with a polynomial fit  
        #ct = 0.2948*J**3 - 0.4303*J**2 + 0.02295*J**1 + 0.07395  Old Fit
        CT = (0.1298*J**3) - 0.2679*J**2 - 0.02553*J + 0.07525
        return CT
    
    def power_coeff(self, J):
        # Overall thrust coefficient for a 20x8 system with a polynomial fit  
        CP = -0.1618*J**3 + 0.0292*J**2 - 0.002126*J + 0.02489
        return CP
    
    def diameter(self):
        return self.diameter

class aeronaut185x12:
    
    def __init__(self):
        self.diameter = 0.4699 # meters

    def efficiency(self, J):
    # Overall efficiency for a 18.5x12 system, with a polynomial fit, find details in onenote
        eff = J * (self.thrust_coeff(J)/self.power_coeff(J))
        return eff
    
    def thrust_coeff(self, J):
    # Overall thrust coefficient for a 18.5x12 system with a polynomial fit  
        #ct = -0.02627*J**3 - 0.1681*J**2 + 0.02741*J**1 + 0.08628
        CT = 0.01649*J**3 - 0.224*J**2 + 0.04329*J + 0.08563
        return CT
    
    def power_coeff(self, J):
        # Overall thrust coefficient for a 20x8 system with a polynomial fit  
        CP = -0.2103*J**3 + 0.1318*J**2 - 0.033076*J + 0.04726
        return CP
    
    def diameter(self):
        return self.diameter