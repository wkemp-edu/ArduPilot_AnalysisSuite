#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 14 17:49:44 2022

@author: williamkemp
"""

# Purpose: Give a better analysis for August data.  
# Mistakes to fix:
    # 1. Bank angle
    # 2. True vs EAS (True is given to us)
    # 3. Better mass estimates
    # 4. Full index calculations, then average result

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import os
from scipy.optimize import curve_fit

from propellers import aeronaut20x8
from motors import U7V2_280KV
import data_importer as imp
import data_exporter as exp
import fittingtools

def func(X, a, b):
    # x is q_inf * V_inf  * S
    # y is V_inf*m*g / (q_inf * cos^2(phi) * pi * AR )
    # a is CDp
    # b is e (Oswald efficiency)
    x,y = X
    return x * a + y / b

# Vehicle parameters
prop = aeronaut20x8() # Using aeronaut 20x8 for this flight
motor = U7V2_280KV()  # Using U7 V2 motor

mass = 12.61797 # Estimated with virtual bolt included
b = 6.28
c = 0.395
S = b*c # m^2
W = mass * 9.807
AR = b**2 / S

P_systems = 4 # Amount of power consumed by everything except the ESC & motor

# Importing ground and flight data
gnd_data = imp.import_gndstation('data/04-08/LOGGER05')
fmt = imp.import_flt('data/04-08/FMT')

august04_loiter = np.array([[1.570e5,1.665e5],
                            [1.668e5,1.782e5],
                            [1.794e5,1.904e5],
                            [1.916e5,2.075e5],
                            [2.087e5,2.166e5],
                            [2.168e5,2.369e5],
                            [2.381e5,2.607e5],
                            [2.619e5,2.671e5],
                            [2.679e5,2.784e5],
                            [2.800e5,3.444e5]])
# Using time from Kalman Filter Output
# Interpolating all other times to the KF

time = fmt.NKF1.TimeS
time = time - time[0]

fmt.CTUN.Aspd = np.interp(time, fmt.CTUN.TimeS, fmt.CTUN.Aspd) # True airspeed (Smoothed from CTUN output)
fmt.ATT.Pitch = np.interp(time, fmt.ATT.TimeS, fmt.ATT.Pitch)
fmt.ATT.Roll = np.interp(time, fmt.ATT.TimeS, fmt.ATT.Roll)
fmt.AOA.AOA = np.interp(time, fmt.AOA.TimeS, fmt.AOA.AOA)
fmt.POS.Alt = np.interp(time, fmt.POS.TimeS, fmt.POS.Alt)
fmt.CESC.RPM = np.interp(time, fmt.CESC.TimeS, fmt.CESC.RPM)
fmt.CESC.Voltage = np.interp(time, fmt.CESC.TimeS, fmt.CESC.Voltage)
fmt.CESC.Curr = np.interp(time, fmt.CESC.TimeS, fmt.CESC.Curr)

# Pressures that are going to be used to find density
Press0 = np.interp(time, fmt.BARO.TimeS, fmt.BARO.Press)
Press1 = np.interp(time, fmt.BAR2.TimeS, fmt.BAR2.Press)
Press = np.mean(np.array([Press0, Press1]), 0) # Pressure measurements averaged

# Temperatures that are going to be used to find density
Temp0 = np.interp(time, fmt.ARSP.TimeS, fmt.ARSP.Temp)
Temp1 = np.interp(time, fmt.ASP2.TimeS, fmt.ASP2.Temp)
Temp = np.mean(np.array([Temp0, Temp1]), 0)

# Used variables:
rho = Press * (287 * (Temp+273.15))**-1             # Density found from barometer pressure & airspeed sensor temperatures
v_eas = fmt.CTUN.Aspd                               # Equivalent SSL airspeed (m/s)
v_tas = v_eas * np.sqrt(1.225) * np.sqrt(rho)**-1   # the true airspeed
n = fmt.CESC.RPM / 60                               # Revolutions per second
P_elec = fmt.CESC.Curr * fmt.CESC.Voltage           # Electrical power to ESC
phi = np.deg2rad(fmt.ATT.Roll)                      # Bank angle in radians

q = 0.5 * rho * v_tas**2                        # Dynamic pressure 
J_tas = v_tas * (n * prop.diameter)**-1             # Advance ratio at TAS
J_tas[J_tas == np.inf] = -1   
eta_motor = motor.efficiency(n, fmt.CESC.Curr)      # Filtering out inf due to n=0
eta_prop = prop.efficiency(J_tas)                   # System efficiency including Zubax ESC
eta_sys = eta_motor * eta_prop

ct = prop.thrust_coeff(J_tas)
T = ct * prop.diameter**4 * rho * n**2              # Getting Thrust from thrust coefficient
P_motor = P_elec                                    # Power consumed by ESC & motor, 
Peta_banked = P_motor * eta_sys                     # Power required to fly at current conditions
Pct_banked = v_tas * T

x = v_tas * q * S
y = (2*W**2) * (rho * v_tas * np.pi * AR * np.cos(phi)**2)**-1

v_dict = {'1':v_tas[int(1.570e5):int(1.665e5)], '2':v_tas[int(1.668e5):int(1.782e5)],
            '3':v_tas[int(1.794e5):int(1.904e5)], '4':v_tas[int(1.916e5):int(2.075e5)],
            '5':v_tas[int(2.087e5):int(2.166e5)], '6':v_tas[int(2.168e5):int(2.369e5)],
            '7':v_tas[int(2.381e5):int(2.486e5)], '8':v_tas[int(2.490e5):int(2.607e5)],
            '9':v_tas[int(2.619e5):int(2.671e5)], '10':v_tas[int(2.679e5):int(2.784e5)], 
            '11':v_tas[int(2.800e5):int(3.444e5)]}

x_dict = {'1':x[int(1.570e5):int(1.665e5)], '2':x[int(1.668e5):int(1.782e5)],
            '3':x[int(1.794e5):int(1.904e5)], '4':x[int(1.916e5):int(2.075e5)],
            '5':x[int(2.087e5):int(2.166e5)], '6':x[int(2.168e5):int(2.369e5)],
            '7':x[int(2.381e5):int(2.486e5)], '8':x[int(2.490e5):int(2.607e5)],
            '9':x[int(2.619e5):int(2.671e5)], '10':x[int(2.679e5):int(2.784e5)], 
            '11':x[int(2.800e5):int(3.444e5)]}

y_dict = {'1':y[int(1.570e5):int(1.665e5)], '2':y[int(1.668e5):int(1.782e5)],
            '3':y[int(1.794e5):int(1.904e5)], '4':y[int(1.916e5):int(2.075e5)],
            '5':y[int(2.087e5):int(2.166e5)], '6':y[int(2.168e5):int(2.369e5)],
            '7':y[int(2.381e5):int(2.486e5)], '8':y[int(2.490e5):int(2.607e5)],
            '9':y[int(2.619e5):int(2.671e5)], '10':y[int(2.679e5):int(2.784e5)], 
            '11':y[int(2.800e5):int(3.444e5)]}

Peta_dict = {'1':Peta_banked[int(1.570e5):int(1.665e5)], '2':Peta_banked[int(1.668e5):int(1.782e5)],
            '3':Peta_banked[int(1.794e5):int(1.904e5)], '4':Peta_banked[int(1.916e5):int(2.075e5)],
            '5':Peta_banked[int(2.087e5):int(2.166e5)], '6':Peta_banked[int(2.168e5):int(2.369e5)],
            '7':Peta_banked[int(2.381e5):int(2.486e5)], '8':Peta_banked[int(2.490e5):int(2.607e5)],
            '9':Peta_banked[int(2.619e5):int(2.671e5)], '10':Peta_banked[int(2.679e5):int(2.784e5)], 
            '11':Peta_banked[int(2.800e5):int(3.444e5)]}

Pct_dict = {'1':Pct_banked[int(1.570e5):int(1.665e5)], '2':Pct_banked[int(1.668e5):int(1.782e5)],
            '3':Pct_banked[int(1.794e5):int(1.904e5)], '4':Pct_banked[int(1.916e5):int(2.075e5)],
            '5':Pct_banked[int(2.087e5):int(2.166e5)], '6':Pct_banked[int(2.168e5):int(2.369e5)],
            '7':Pct_banked[int(2.381e5):int(2.486e5)], '8':Pct_banked[int(2.490e5):int(2.607e5)],
            '9':Pct_banked[int(2.619e5):int(2.671e5)], '10':Pct_banked[int(2.679e5):int(2.784e5)], 
            '11':Pct_banked[int(2.800e5):int(3.444e5)]}

x_ave = np.zeros(11)
y_ave = np.zeros(11)
v_ave = np.zeros(11)
Peta_ave = np.zeros(11)
Pct_ave = np.zeros(11)

for index, set_num in enumerate(x_dict):
    v_ave[index] = np.mean(v_dict.get(str(set_num)))
    x_ave[index] = np.mean(x_dict.get(str(set_num)))
    y_ave[index] = np.mean(y_dict.get(str(set_num)))
    Peta_ave[index] = np.mean(Peta_dict.get(str(set_num)))
    Pct_ave[index] = np.mean(Pct_dict.get(str(set_num)))

results = curve_fit(func, (x_ave, y_ave), Peta_ave)
CDo_eta = results[0][0]
e_eta = results[0][1]

results = curve_fit(func, (x_ave, y_ave), Pct_ave)
CDo_ct = results[0][0]
e_ct = results[0][1]

CDeta = Peta_banked / (v_tas * q * S)
CDct = Pct_banked / (v_tas * q * S)

CL = (W/np.cos(phi))/(q*S)
CLsq = CL**2

plt.figure(figsize=(10,5))
for i in range(len(august04_loiter)-1):
    print(i)
    plt.scatter(CLsq[int(august04_loiter[i, 0]):int(august04_loiter[i, 1])], CDeta[int(august04_loiter[i, 0]):int(august04_loiter[i, 1])], label="Eta Method")
    #plt.scatter(CL[int(august04_loiter[i, 0]):int(august04_loiter[i, 1])], CDct[int(august04_loiter[i, 0]):int(august04_loiter[i, 1])], label="CT Method")
plt.legend()
plt.show()

# Finding CL, compensated for banked flight
savename = str(os.path.basename(__file__))
savename = savename[0:-3]

