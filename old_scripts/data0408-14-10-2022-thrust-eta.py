#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 14 17:49:44 2022

@author: williamkemp
"""

# Purpose: Give a better analysis for August data.  
# Mistakes to fix:
    # 1. Bank angle
    # 3. Better mass estimates
    # 3. Full index calculations, then average result

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import os
from scipy.optimize import curve_fit

from propellers import aeronaut20x8
from motors import U7V2_280KV
from aircraft import airplane

import data_importer as imp
import data_exporter as exp
import fittingtools
import cl_finders

# Vehicle parameters
prop = aeronaut20x8() # Using aeronaut 20x8 for this flight
motor = U7V2_280KV()  # Using U7 V2 motor

mass = 12.61797 # Estimated with virtual bolt included
span = 6.28
chord = 0.395
createv = airplane(mass, chord, span)

P_systems = 4 # Amount of power consumed by everything except the ESC & motor

# Importing ground and flight data
gnd_data = imp.import_gndstation('data/04-08/LOGGER05')
fmt = imp.import_flt('data/04-08/FMT')

mask = np.array([[1.570e5,1.665e5],
                [1.668e5,1.782e5],
                [1.794e5,1.904e5],
                [1.916e5,2.075e5],
                [2.087e5,2.166e5],
                [2.168e5,2.369e5],
                [2.381e5,2.486e5],
                [2.490e5,2.607e5],
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

############# Main Analysis ###############3

# Attitude 
phi = np.deg2rad(fmt.ATT.Roll)                      # Bank angle in radians

# Atmospheric adjustments:
rho = Press * (287 * (Temp+273.15))**-1             # Density found from barometer pressure & airspeed sensor temperatures
v_eas = fmt.CTUN.Aspd                               # Equivalent SSL airspeed (m/s)
v_tas = v_eas * np.sqrt(1.225) * np.sqrt(rho)**-1   # the true airspeed
q = 0.5 * rho * v_tas**2                            # Dynamic pressure 

# Propulsion characterization
n = fmt.CESC.RPM / 60                               # Revolutions per second
i_esc = fmt.CESC.Curr
v_esc = fmt.CESC.Voltage

# Estimated propulsive power
P_eta = cl_finders.eta_steady(prop, motor, v_tas, n, i_esc, v_esc)
P_ct = cl_finders.thrust_steady(prop, rho, v_tas, n)

# Getting drag coefficient
Cd_eta = cl_finders.preq2cd(createv, v_tas, q, P_eta)
Cd_ct = cl_finders.preq2cd(createv, v_tas, q, P_ct)

# Getting lift coefficient
CL = cl_finders.cl_banked(createv, q, phi)


# Plot with static masking

plt.figure(figsize=(10,5))
plt.title("Manual Masking")

markers = ['^', 'o', '<', '>', '*', 's', 'v', '+', 'x', 'D', '.']

for i in range(len(mask)):
    plt.plot(CL[int(mask[i,0]):int(mask[i,1])], Cd_eta[int(mask[i,0]):int(mask[i,1])], linestyle='', marker=markers[i], markerfacecolor='r', markersize=4, markeredgecolor=[0,0,0])
    plt.plot(CL[int(mask[i,0]):int(mask[i,1])], Cd_ct[int(mask[i,0]):int(mask[i,1])], linestyle='', marker=markers[i], markerfacecolor='b', markersize=4, markeredgecolor=[0,0,0])
plt.grid(True)
plt.xlabel("Lift Coefficient")
plt.ylabel("Drag Coefficient")
plt.show()

plt.figure(figsize=(10,5))
plt.title("Power to Propulsion Estimates")
for i in range(len(mask)):
    plt.plot(v_tas[int(mask[i,0]):int(mask[i,1])], P_eta[int(mask[i,0]):int(mask[i,1])], label="Index: "+str(i), linestyle='', marker=markers[i], markerfacecolor='r', markersize=4, markeredgecolor=[0,0,0])
    plt.plot(v_tas[int(mask[i,0]):int(mask[i,1])], P_ct[int(mask[i,0]):int(mask[i,1])], linestyle='', marker=markers[i], markerfacecolor='b', markersize=4, markeredgecolor=[0,0,0])
plt.grid(True)
plt.xlabel("True Airspeed (m/s)")
plt.ylabel("Propulsive Power (W)")
plt.legend()
plt.show()

CDct_ave = np.zeros(11)
CDeta_ave = np.zeros(11)
CL_ave = np.zeros(11)
EAS_ave = np.zeros(11)

CDct_std = np.zeros(11)
CDeta_std = np.zeros(11)
CL_std = np.zeros(11)

for i in range(len(mask)):
    print(i)
    print(int(mask[i,0]))
    print(int(mask[i,1]))
    
    CDeta_ave[i] = np.mean(Cd_eta[int(mask[i,0]):int(mask[i,1])])
    CDct_ave[i] = np.mean(Cd_ct[int(mask[i,0]):int(mask[i,1])])
    CL_ave[i] = np.mean(CL[int(mask[i,0]):int(mask[i,1])])
    EAS_ave[i] = np.mean(v_eas[int(mask[i,0]):int(mask[i,1])])
    
    CDeta_std[i] = np.std(Cd_eta[int(mask[i,0]):int(mask[i,1])])
    CDct_std[i] = np.std(Cd_ct[int(mask[i,0]):int(mask[i,1])])
    CL_std[i] = np.std(CL[int(mask[i,0]):int(mask[i,1])])
    
# Finding drag polar:
polar_eta = cl_finders.cd2polar(createv, CDeta_ave, CL_ave)
polar_ct = cl_finders.cd2polar(createv, CDct_ave, CL_ave)

# Finding power required @ SSL @ Standard weight of 12.6 kg
Peq_eta, EAS = cl_finders.polar2preqew(createv, polar_eta, (7,14))
Peq_ct, EAS = cl_finders.polar2preqew(createv, polar_ct, (7,14))

plt.figure(figsize=(10,5))
plt.plot(EAS, Peq_ct, label="Thrust Coefficient Method")
plt.plot(EAS, Peq_eta, label="Eta Coefficient Method")
plt.scatter(EAS_ave, CDct_ave * 0.5 * 1.225 * EAS_ave**3 * createv.area, marker='^', label='CT Method Points')
plt.scatter(EAS_ave, CDeta_ave * 0.5 * 1.225 * EAS_ave**3 * createv.area, marker='^', label='Eta Method Points')
plt.xlabel("Airspeed (Standard Sea-Level)")
plt.ylabel("Power Required at Standard Mass (12.6 KG)")
plt.legend()
plt.grid(True)
plt.show()
    
plt.figure(figsize=(10,5))
plt.errorbar(CDeta_ave, CL_ave, xerr=CDeta_std, yerr=CL_std, fmt='+b', label="Eta Method")
plt.errorbar(CDct_ave, CL_ave, xerr=CDct_std, yerr=CL_std, fmt='xr', label="CT Method")
plt.xlabel("Drag Coefficient")
plt.ylabel("Lift Coefficient")
plt.legend()
plt.grid(True)
plt.show()

# Dynamic Masking

