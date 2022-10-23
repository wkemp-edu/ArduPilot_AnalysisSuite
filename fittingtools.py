#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 28 03:16:43 2022

@author: williamkemp
"""

import pandas as pd
import numpy as np

def fit_P(v, a, b):
    # Fitting 
    result = a*v**3 + b*v**-1
    return result

def fit_CLa(x, a, b, c):
    
    # Fitting the CL vs alpha function based on 2nd order polynomial
    result = a*x**2 + b*x**1 + c*x**0
    return result

def export_csv(EAS, CD, CL):
    # inputs are all numpy arrays of equal size
    return None
    
def fit_dpress2drag(dpress, CDp, H):
    # Fitting drag as function of dynamic pressure
    # CDp is the profile drag coefficient
    # H is W^2 / (pi * AR * e)
    result = (CDp * dpress) + (H * dpress**-1)
    return result
    