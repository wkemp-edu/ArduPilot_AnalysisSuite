#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  4 14:34:44 2021

@author: williamkemp
"""

import pandas as pd
from scipy import io
import numpy as np
import fittingtools
from scipy import optimize as op
from pathlib import Path

def export_polar(name, P_req_ssl, V_ssl, mass, AR, S, phi, alpha=False):
    
    # Inputs:
        # 0. name := name of the csv file exported!
        # 1. P_req_ssl := power required at SSL
        # 2. V_ssl := equivalent airspeed at SSL
        # 3. alpha := Angle of attack in radians
        # 4. mass := Best estimate of vehicle mass
        # 5. AR := Best estimate of vehicle aspect ratio
    # Outputs:
        # 1. CL := coefficient of lift
        # 2. CDp := coefficient of profile drag
        # 3. e := Oswald efficiency (induced drag coeff parameter)
    # Files:  
        # Results in CSV under same script name with outputs as columns
    # Takes the determined required power, airspeed and finds the non-dim values to export
    
    g = 9.807 # Gravitational acceleration
    rho_ssl = 1.225 # Standard sea-level density
    q_ssl = 0.5 * rho_ssl * V_ssl**2
    
    W = mass * g    
    
    # Getting CL & alpha
    CL_data = W * (np.cos(phi))**-1 * (q_ssl * S)**-1
    
    # Getting CDp
    params_Pfit, _ = op.curve_fit(fittingtools.fit_P, V_ssl, P_req_ssl)
    
    v = np.linspace(8,14,100)
    CL = W/(0.5 * rho_ssl * v**2 * S)

    
    CDp = params_Pfit[0] * (S * 0.5 * rho_ssl)**-1 # Finding profile drag coefficient
    CDi = params_Pfit[1] * (0.5 * (rho_ssl * v**4 * S))**-1 # Finding induced drag coefficient
    CD = CDi + CDp
    
    # Getting e
    e = (CL ** 2 / CDi) * (np.pi * AR)**-1
    
    # Coefficient of lift alpha 
    if type(alpha)!=bool:
        params_CLa, _ = op.curve_fit(fittingtools.fit_CLa, CL_data, alpha)
        AoA = params_CLa[0]*CL**2 + params_CLa[1]*CL**1 + params_CLa[2]*CL**0
        d = {'CL': CL, 'CDp': CDp, 'CDi': CDi, 'e': e, 'AoA': AoA}
    else:
        d = {'CL': CL, 'CDp': CDp, 'CDi': CDi, 'e': e}
    
    # Creating dataframe
    df = pd.DataFrame(d)
    
    # Setting airspeed (SSL) as index
    df.index = list(v)
    df.index.name = 'EAS'
    
    # Saving results to CSV in Results folder
    resultspath = Path('Results/'+name+'.csv')
    df.to_csv(resultspath, index=True)
    
    return df
