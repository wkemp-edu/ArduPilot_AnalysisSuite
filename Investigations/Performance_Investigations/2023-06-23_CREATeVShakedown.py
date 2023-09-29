# %%
### Definining name of analysis ###
name = '2023-06-23_CREATeVShakedown'
data_path = '../data/'
data_folder = 'createv-2023-06-23'
file_name = '00000012.BIN'
datasave_path = data_path+data_folder+'/'
result_path = '../Results/'+name+'/'
figure_path = '../Figures/'+name+'/'

# %%
from os import sys
import os
sys.path.append('../')

# Getting packages #
%matplotlib widget

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from scipy import optimize 
from scipy.io import savemat

import plotly.express as px

import main
from main import flight
from main import analysis
import propellers
import motors
from aircraft import airplane
import cl_finders

import data_exporter
import datetime

### Creating Folders for Results/Data ###
if not os.path.exists(result_path):
    os.makedirs(result_path)
if not os.path.exists(figure_path):
    os.makedirs(figure_path)

# %%
# Setting Plot Defaults
plt.style.use('../basic_plotter.mplstyle')

# %%
# Importing data, specifying import characteristics, backing up results
rate = "10ms"
interpolateM = "linear"
processor = "CREATeV_2023"

df = main.data_load(data_path, data_folder, file_name, rate, interpolateM, processor)
# rawdata_name = data_folder+'/'+file_name.split('.')[0]+'_'+rate+'_'+interpolateM+'_'+processor+'.pkl'

# # Checking if the data is already pickled for analysis
# if os.path.exists(data_path+rawdata_name):
#     df = pd.read_pickle(data_path+rawdata_name)
# else:
#     df = main.get_data(processor, data_folder+'/'+file_name, rate)
#     df = df.interpolate(method=interpolateM)
#     pd.to_pickle(df, data_path+rawdata_name) # Storing parsed data

# %%
# Vehicle parameters
prop = propellers.aeronaut20x8() # Using aeronaut 20x8 for this flight
motor = motors.U7V2_280KV()  # Using U7 V2 motor

mass = 12.502 # Estimated with virtual bolt included
span = 6.28
chord = 0.395
createv = airplane(mass, chord, span)

P_systems = 0.3 # Amount of power consumed by everything except the ESC & motor

highorderpolar = True

# %%
# Generating Plots
timeindex_v_thr_h = plt.figure(figsize=(10,10))

ax0 = plt.subplot(3,1,1)
plt.grid("On")
plt.plot(df.index, df.Airspeed_Sensor0, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
plt.ylabel("EAS Airspeed (m/s)")
ax1 = plt.subplot(3,1,2, sharex=ax0)
plt.grid("On")
plt.plot(df.index, df.ThrottleOut, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
plt.plot(df.index, df.Throttle, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
plt.ylabel("Throttle Outputs")
ax2 = plt.subplot(3,1,3, sharex=ax0)
plt.grid("On")
plt.plot(df.index, df.Altitude_BARO_0, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
plt.ylabel("Barometric Altitude (h)")
plt.xlabel("Time of Day")
plt.grid("On")
plt.show()

#main.save_figure(timeindex_v_thr_h, f'{timeindex_v_thr_h=}'.split('=')[0], figure_path)

# %%
# Plotting raw IMU accelerations
raw_IMU0_accelerations = plt.figure(figsize=(5.5,7))
ax0 = plt.subplot(3,1,1)
plt.plot(df.index, df.XAcc_IMU0, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
plt.ylabel("X Acceleration $(m/s^2)$")
ax1 = plt.subplot(3,1,2, sharex=ax0)
plt.plot(df.index, df.YAcc_IMU0, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
plt.ylabel("Y Acceleration $(m/s^2)$")
ax2 = plt.subplot(3,1,3, sharex=ax0)
plt.plot(df.index, df.ZAcc_IMU0, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
plt.xlabel("Time of Day")
plt.ylabel("Z Acceleration $(m/s^2)$")
plt.show()

# main.save_figure(raw_IMU0_accelerations, f'{raw_IMU0_accelerations=}'.split('=')[0], figure_path)

# %%
# Plotting Propulsion Information
timeindex_I_RPM_thr = plt.figure(figsize=(5.5,7))

ax0 = plt.subplot(3,1,1)
ax0.plot(df.index, df.MainBatteryCurrent, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
ax0.set_ylabel("ESC Current (A)")
ax1 = plt.subplot(3,1,2, sharex=ax0)
ax1.plot(df.index, df.MotorRPM, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
ax1.set_ylabel("Motor RPM")
ax2 = plt.subplot(3,1,3, sharex=ax0)
ax2.plot(df.index, df.ThrottleOut, linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
ax2.set_ylabel("PWM Throttle")
ax2.set_xlabel("Time of Day")
plt.show()

#main.save_figure(timeindex_I_RPM_thr, f'{timeindex_I_RPM_thr=}'.split('=')[0], figure_path)

# %%
### Mask Initialization ###

# Generating Start/End Times for Segments #
# Producing mask_array for dataframe

# def get_mask(df, start, end, year, month, day):
#     # Getting boolean mask from start and end times
#     start_time = get_datetime(start, year, month, day)
#     end_time = get_datetime(end, year, month, day)
#     mask = (df.index > start_time) & (df.index < end_time)
#     return mask

# def get_datetime(hour_string, year, month, day):
#     # Results completed datetime from hour string, and date
#     split_nums = hour_string.split(':')
#     hours = int(split_nums[0])
#     minutes = int(split_nums[1])
#     seconds = int(split_nums[2])
#     return pd.Timestamp(year=year, month=month, day=day, hour=hours, minute=minutes, second=seconds)

year = 2023
month = 6
day = 22

# Acceleration Masking from the above figures
seg_times = np.array([['12:27:16','12:27:26'],
                      ['12:29:50','12:30:10'],
                      ['12:32:24','12:32:44'],
                      ['12:34:54','12:35:20'],
                      ['12:37:15','12:37:40'],
                      ['12:39:26','12:39:51'],
                      ['12:41:29','12:41:54'],
                      ['12:43:45','12:44:10'],
                      ['12:45:52','12:46:16'],
                      ['12:48:20','12:48:40'],
                      ['12:50:36','12:50:59'],
                      ['12:52:46','12:53:05'],
                      ['12:55:10','12:55:34'],
                      ['12:57:20','12:57:49'],
                      ['12:59:44','13:00:06'],
                      ['13:02:05','13:02:30'],
                      ['13:04:17','13:04:42'],
                      ['13:06:25','13:06:47'],
                      ['13:08:39','13:08:42'],
                      ['13:10:29','13:10:42'],
                      ['13:12:13','13:12:28'],
                      ['13:14:29','13:14:47'],
                      ['13:16:36','13:16:55']])
seg_zeroing = np.array([['9:27:39','9:41:45']])

mask_array = cl_finders.get_maskarray(df, seg_times, year, month, day)
acc_masks = cl_finders.get_maskarray(df, seg_zeroing, year, month, day)

# mask_array = []
# acc_masks = []
# for i in range(np.shape(seg_times)[0]):
#     mask = cl_finders.get_mask(df, seg_times[i,0], seg_times[i,1], year, month, day)
#     mask_array.append(mask)
# for i in range(np.shape(seg_zeroing)[0]):
#     mask = cl_finders.get_mask(df, seg_zeroing[i,0], seg_zeroing[i,1], year, month, day)
#     acc_masks.append(mask)

# %%
############# Main Analysis ###############3

# Gravity
g = 9.807

# Attitude 
phi = np.deg2rad(df["RollAngle"].to_numpy())            # Bank angle in radians
theta = np.deg2rad(df["PitchAngle"].to_numpy())         # Pitch angle in radians

# Atmospheric adjustments:
rho = df["Pressure_BARO0"].to_numpy() * (287 * (df["Temperature_ARSP"].to_numpy()+273.15))**-1             # Density found from barometer pressure & airspeed sensor temperatures
v_eas = df["Airspeed_Sensor0"].to_numpy()                               # Equivalent SSL airspeed (m/s)
v_tas = v_eas * np.sqrt(1.225) * np.sqrt(rho)**-1   # the true airspeed
q = 0.5 * rho * v_tas**2                            # Dynamic pressure 

# For Descent method
h = df["Altitude_POS"].to_numpy()                      # Altitude
Vd_eas = df["DescentRate"].to_numpy()                  # Descent Rate from EKF (is it true or EAS at SSL?)
Vd_tas = Vd_eas * np.sqrt(1.225) * np.sqrt(rho)**-1    # the true airspeed
gamma = np.arcsin(-Vd_tas/v_tas) ####Should this be negative or positive
alpha = gamma + theta

# Propulsion characterization
n = df["MotorRPM"].to_numpy() / 60                      # Revolutions per second
i_esc = df["MainBatteryCurrent"].to_numpy()             # Really the ESC voltage and current here
v_esc = df["MainBatteryVoltage"].to_numpy()
J = v_tas / (n * prop.diameter)
eff = prop.efficiency(J) * motor.efficiency(n, i_esc)

# Inertial Measurement Unit
xp_acc = df.XAcc_IMU0.to_numpy()                       # Acceleration in X direction of the IMU
yp_acc = df.YAcc_IMU0.to_numpy()                       # Acceleration in Y direction of the IMU
zp_acc = df.ZAcc_IMU0.to_numpy()                       # Acceleration in Z direction of the IMU

U_dot = xp_acc - g * np.sin(theta)
V_dot = yp_acc + (9.807 * np.cos(theta) * np.sin(phi))
W_dot = zp_acc + g * np.cos(theta) * np.cos(phi)

gamma = np.arcsin(-Vd_tas/v_tas)
alpha = gamma + theta

U = v_tas * np.cos(alpha)
V = 0
W = v_tas * np.sin(alpha)

P = df.GyroX_IMU0 
Q = df.GyroY_IMU0
R = df.GyroZ_IMU0

# %%
plt.figure(figsize=(10,10))
plt.tight_layout()
t_init = 0
markerevery=80
markedgew=0.5

ax1 = plt.subplot(4,1,1)
ax2 = plt.subplot(4,1,2, sharex=ax1)
ax3 = plt.subplot(4,1,3, sharex=ax1)
ax4 = plt.subplot(4,1,4, sharex=ax1)
for i in range(len(mask_array)):
    dT = 0.01
    segment_length = len(v_tas[mask_array[i]])            # Integer segment length
    time_s = np.linspace(t_init, (segment_length-1)*dT + t_init, segment_length)
    ax1.plot(time_s, P[mask_array[i]], color='r', markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, label='P')
    ax1.plot(time_s, Q[mask_array[i]], color='b', markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, label='Q')
    ax1.plot(time_s, R[mask_array[i]], color='g', markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, label='R')
    if i == 0:
        ax1.legend()
    ax2.plot(time_s, v_tas[mask_array[i]], color='r', markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    ax2.set_ylabel("$V_T$")
    ax3.plot(time_s, np.rad2deg(alpha[mask_array[i]]), color='r', markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    ax3.set_ylabel("$\\alpha$")
    ax4.plot(time_s, df.ElevatorOut[mask_array[i]], color='g', markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    ax4.set_ylabel("Elevator PWM")
    ax4.set_xlabel("Time (s)")
    t_init = time_s[-1]+10
plt.show()

# %%
# Generating FFT of rotation rates

# Number of samplepoints
# sample spacing
x = np.array([0.1, 0.1, 0.2, 0.3])
dT = 0.01
y = np.linspace(0.5, (len(x)-1)*dT + 0.5, len(x))
print(y)

i = 7

segment_length = len(v_tas[mask_array[i]])            # Integer segment length
time_s = np.linspace(t_init, (segment_length-1)*dT + t_init, segment_length)

plt.figure()
plt.plot(time_s, P[mask_array[i]])
plt.show()

p_fft = np.fft.fft(P[mask_array[i]])
q_fft = np.fft.fft(Q[mask_array[i]])
r_fft = np.fft.fft(R[mask_array[i]])

xf = np.linspace(0.0, 1.0/(2.0*dT), segment_length//2)

plt.figure()
ax0 = plt.subplot(3,1,1)
ax1 = plt.subplot(3,1,2)
ax2 = plt.subplot(3,1,3)
ax0.plot(xf, 2.0/segment_length * np.abs(p_fft[:segment_length//2]))
ax1.plot(xf, 2.0/segment_length * np.abs(q_fft[:segment_length//2]))
ax2.plot(xf, 2.0/segment_length * np.abs(r_fft[:segment_length//2]))
plt.show()

plt.figure()
plt.plot(time_s, np.fft.ifft(p_fft))
plt.show()

# %%
plt.figure(dpi=150)
plt.hist(df.MotorRPM[mask_array[i]]/60, bins=30, edgecolor='black')
plt.xlabel("Motor Rev/s")
plt.figure()

# %%
thetap = 1.0 * (np.pi/180)  # Pitch perturbation from IMU orientation (Degrees)
phip = 0.0 * (np.pi/180)  # Roll perturbation from IMU orientation (Degrees)
IMUtotal = np.sqrt(xp_acc**2 + yp_acc**2 + zp_acc**2)
print(np.mean(IMUtotal[acc_masks[0]]))
print(np.std(IMUtotal[acc_masks[0]]))

U_dot = xp_acc - np.mean(IMUtotal[acc_masks[0]]) * np.sin(theta)
V_dot = yp_acc + (np.mean(IMUtotal[acc_masks[0]]) * np.cos(theta) * np.sin(phi))
W_dot = zp_acc + np.mean(IMUtotal[acc_masks[0]]) * np.cos(theta) * np.cos(phi)

U_dotp = xp_acc - (np.mean(IMUtotal[acc_masks[0]]) * np.sin(theta+thetap))
V_dotp = yp_acc + (np.mean(IMUtotal[acc_masks[0]]) * np.cos(theta+thetap) * np.sin(phi+phip))
W_dotp = zp_acc + (np.mean(IMUtotal[acc_masks[0]]) * np.cos(theta+thetap) * np.cos(phi+phip))

def plot_hist(mask_array):

    plt.figure(figsize=(5.5, 7))
    plt.title("Body Accelerations in X Direction")
    i = 0
    ax0 = plt.subplot(3,1,1)
    ax0.hist(U_dotp[mask_array[i]], bins=50, edgecolor='k', label='Adjusted')
    ax0.hist(U_dot[mask_array[i]], bins=50, edgecolor='k', label='Original')
    ax0.axvline(np.mean(U_dotp[mask_array[i]]), color='k', linestyle='dashed', linewidth=2)
    ax0.axvline(np.mean(U_dot[mask_array[i]]), color='r', linestyle='dashed', linewidth=2)
    ax0.set_xlabel("$\dot{U}$")
    min_ylim, max_ylim = ax0.set_ylim()
    ax0.text(U_dotp[mask_array[i]].mean()*1.1, max_ylim*0.9, 'Mean: {:.5f}'.format(U_dotp[mask_array[i]].mean()))
    ax0.text(U_dot[mask_array[i]].mean()*1.1, max_ylim*0.9, 'Mean: {:.5f}'.format(U_dot[mask_array[i]].mean()))

    ax1 = plt.subplot(3,1,2)
    ax1.hist(V_dotp[mask_array[i]], bins=50, edgecolor='k', label='Adjusted')
    ax1.hist(V_dot[mask_array[i]], bins=50, edgecolor='k', label='Original')
    ax1.axvline(np.mean(V_dotp[mask_array[i]]), color='k', linestyle='dashed', linewidth=2)
    ax1.axvline(np.mean(V_dot[mask_array[i]]), color='r', linestyle='dashed', linewidth=2)
    ax1.set_xlabel("$\dot{V}$")
    min_ylim, max_ylim = ax1.set_ylim()
    ax1.text(V_dotp[mask_array[i]].mean()*1.1, max_ylim*0.9, 'Mean: {:.5f}'.format(V_dotp[mask_array[i]].mean()))
    ax1.text(V_dot[mask_array[i]].mean()*1.1, max_ylim*0.9, 'Mean: {:.5f}'.format(V_dot[mask_array[i]].mean()))

    ax2 = plt.subplot(3,1,3)
    ax2.hist(W_dotp[mask_array[i]], bins=50, edgecolor='k', label='Adjusted')
    ax2.hist(W_dot[mask_array[i]], bins=50, edgecolor='k', label='Original')
    ax2.axvline(np.mean(W_dotp[mask_array[i]]), color='k', linestyle='dashed', linewidth=2)
    ax2.axvline(np.mean(W_dot[mask_array[i]]), color='r', linestyle='dashed', linewidth=2)
    ax2.set_xlabel("$\dot{W}$")
    min_ylim, max_ylim = ax2.set_ylim()
    ax2.text(W_dotp[mask_array[i]].mean()*1.1, max_ylim*0.9, 'Mean: {:.5f}'.format(W_dotp[mask_array[i]].mean()))
    ax2.text(W_dot[mask_array[i]].mean()*1.1, max_ylim*0.9, 'Mean: {:.5f}'.format(W_dot[mask_array[i]].mean()))

    plt.show()
plot_hist(acc_masks)


# %%
# Beginning Power Required Calculation #
CT = prop.thrust_coeff(J)
T = CT * rho * n**2 * prop.diameter**4
P_req_t1 = (T * v_tas)
P_req_t2 = (mass * g * Vd_tas)
P_req_t3 = -(mass * U_dot * v_tas)
P_req_t4 = -(mass * W_dot * v_tas * alpha)

P_req_q = mass * v_tas * (Q*U*alpha - Q*W)
P_req_pr = mass * v_tas * (R*V - P*V)

P_req = P_req_t1 + P_req_t2 + P_req_t3
P_req_r1 = P_req_t1 + P_req_t2 + P_req_t3 + P_req_t4
P_req_r2 = P_req_t1 + P_req_t2 + P_req_t3 + P_req_t4 + P_req_q + P_req_pr

D = P_req / v_tas
D_r1 = P_req_r1 / v_tas
D_r2 = P_req_r2 / v_tas

L = mass * (U_dotp * alpha - W_dotp) + (mass * g) - (T * alpha)

CD = D / (0.5 * rho * v_tas**2 * createv.area) # still need to estimate the wing area of createv
CD_r1 = D_r1 / (0.5 * rho * v_tas**2 * createv.area) # still need to estimate the wing area of createv
CD_r2 = D_r2 / (0.5 * rho * v_tas**2 * createv.area) # still need to estimate the wing area of createv
CL = cl_finders.cl_usbanked(createv, q, phi, W_dot)
CL_exp = L / (0.5 * rho * v_tas**2 * createv.area)

# %%
# Using new functions to clean up the results

cl_total = cl_finders.total_segments_boolean(CL, mask_array)
cd_total_r2 = cl_finders.total_segments_boolean(CD_r2, mask_array)
cd_total_r1 = cl_finders.total_segments_boolean(CD_r1, mask_array)
aoa_total = cl_finders.total_segments_boolean(alpha, mask_array)

bins = np.linspace(0.2, 1.4, 50)
[cl_bin_means, cl_bin_stds, cl_bin_ci95s, cd_bin_means, cd_bin_stds, cd_bin_ci95s] = cl_finders.collect_bins(bins, cl_total, cd_total_r2)
[cl_bin_means, cl_bin_stds, cl_bin_ci95s, aoa_bin_means, aoa_bin_stds, aoa_bin_ci95s] = cl_finders.collect_bins(bins, cl_total, aoa_total)


plt.figure()
plt.plot(cd_total_r2, cl_total, linestyle='', marker='o', alpha=0.1)
plt.plot(cd_bin_means, cl_bin_means, marker='')
plt.show()

# %%
# Plotting the percent change from the PQR rates in the transient response
fig = plt.figure(figsize=(7.7,5))
ax = plt.subplot(1,1,1)
percent_change = 100 * (P_req_r2 - P_req_r1) / P_req_r1
for i in range(len(mask_array)):
    ax.hist(percent_change[mask_array[i]], bins=np.arange(-1, 1, 0.05), edgecolor=[0,0,0], label='Run_'+str(i), alpha=0.3)
plt.xlabel("P_req Percent change from PQR Rates")
plt.legend()
plt.show()

fig = plt.figure(figsize=(7.7,5))
ax = plt.subplot(1,1,1)
for i in [3,4,5,6,7]:
    ax.hist(CL_exp[mask_array[i]], bins=np.arange(-0.1, 1, 0.05), edgecolor=[0,0,0], label='Run_'+str(i), alpha=0.3)
plt.xlabel("Coefficient of Lift Samples")
plt.legend()
plt.show()

fig = plt.figure(figsize=(7.7,5))
ax = plt.subplot(1,1,1)
for i in []:
    ax.hist(CL_exp[mask_array[i]], bins=np.arange(-0.1, 1, 0.05), edgecolor=[0,0,0], label='Run_'+str(i), alpha=0.3)
plt.xlabel("Coefficient of Lift Samples")
plt.legend()
plt.show()

# %%
# Plotting the Rotational Rates
markerstyle = 'o'
markerevery = 20
markedgew = 0.05

t_init = 0

plt.figure(figsize=(10,10))
plt.tight_layout()
t_init = 0

ax1 = plt.subplot(3,1,1)
ax2 = plt.subplot(3,1,2, sharex=ax1)
ax3 = plt.subplot(3,1,3, sharex=ax1)
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax1.plot(time_s, P[mask_array[i]], color='r', marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, label='P')
    ax1.plot(time_s, Q[mask_array[i]], color='b', marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, label='Q')
    ax1.plot(time_s, R[mask_array[i]], color='g', marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, label='R')
    ax2.plot(time_s, v_tas[mask_array[i]], color='r', marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    ax3.plot(time_s, np.rad2deg(alpha[mask_array[i]]), color='r', marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax1.grid(True)
ax2.grid(True)
ax1.legend()
plt.show()

# %%
# Plotting Masked Acceleration Run Variables
markerstyle = 'o'
markerevery = 80
markedgew = 0.05
plt.figure(figsize=(5.5,6.5))
ax1 = plt.subplot(4,1,1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax1.plot(time_s, v_tas[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax1.set_ylabel("True Airspeed (m/s)")

ax0 = plt.subplot(4,1,2, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax0.plot(time_s, Vd_tas[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax0.set_ylabel("Climb Rate $\dot{h}$ (m/s)")

ax2 = plt.subplot(4,1,3, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax2.plot(time_s, U_dot[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax2.set_ylabel("$\dot{U}$ (m/s^2)")

ax3 = plt.subplot(4,1,4, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax3.plot(time_s, W_dot[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax3.set_ylabel("$\dot{W}$ (m/s^2)")
ax3.set_xlabel("Time $t$ (s)")
plt.show()

# %%
# Plotting Acceleration Runs
markerstyle = 'o'
markerevery = 80
markedgew = 0.05
plt.figure(figsize=(7,7))
ax1 = plt.subplot(4,1,1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax1.plot(time_s, J[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax1.set_ylabel("Advance Ratio")
ax1.grid(which='major', linestyle='-')
ax1.grid(which='minor', linestyle=':', color='grey')

ax0 = plt.subplot(4,1,2, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax0.plot(time_s, v_tas[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax0.set_ylabel("True Airspeed $V_{TAS}$ (m/s)")
ax0.grid(which='major', linestyle='-')
ax0.grid(which='minor', linestyle=':', color='grey')

ax2 = plt.subplot(4,1,3, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax2.plot(time_s, n[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax2.set_ylabel("rev/s")
ax2.grid(which='major', linestyle='-')
ax2.grid(which='minor', linestyle=':', color='grey')

ax3 = plt.subplot(4,1,4, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax3.plot(time_s, P_req[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    ax3.plot(time_s, P_req_r2[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, label='Angle of Attack Rotated')
    t_init = time_s[-1]+10
#ax3.legend()
ax3.set_ylabel("$P_{req}$ (N)")
ax3.set_xlabel("Time $t$ (s)")
ax3.grid(which='major', linestyle='-')
ax3.grid(which='minor', linestyle=':', color='grey')



plt.show()

# %%
markerstyle = 'o'
markerevery = 1
markedgew = 0.1
plt.figure(figsize=(14,7))
ax1 = plt.subplot(1,1,1)
for i in range(len(mask_array)):
    ax1.plot(v_tas[mask_array[i]], CT[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='')
ax1.set_ylabel("Thrust Coefficient (W)")
ax1.set_xlabel("True Airspeed (m/s)")
ax1.grid(which='major', linestyle='-')
ax1.grid(which='minor', linestyle=':', color='grey')
ax1.autoscale(enable=True, axis='x', tight=True)
plt.show()

# %%
# Plotting Angle of Attack Parameters
markerstyle = 'o'
markerevery = 80
markedgew = 0.05
plt.figure(figsize=(7,7))
ax1 = plt.subplot(4,1,1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax1.plot(time_s, v_tas[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax1.set_ylabel("True Airspeed $V_{TAS}$ (m/s)")
ax1.grid(which='major', linestyle='-')
ax1.grid(which='minor', linestyle=':', color='grey')
ax1.autoscale(enable=True, axis='x', tight=True)

ax0 = plt.subplot(4,1,2, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax0.plot(time_s, np.rad2deg(gamma[mask_array[i]]), marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax0.set_ylabel("Flight Path Angle $\gamma$ [deg]")
ax0.grid(which='major', linestyle='-')
ax0.grid(which='minor', linestyle=':', color='grey')
ax0.autoscale(enable=True, axis='x', tight=True)

ax2 = plt.subplot(4,1,3, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax2.plot(time_s, np.rad2deg(alpha[mask_array[i]]), marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax2.set_ylabel("Angle of Attack (deg)")
ax2.grid(which='major', linestyle='-')
ax2.grid(which='minor', linestyle=':', color='grey')
ax2.autoscale(enable=True, axis='x', tight=True)

ax3 = plt.subplot(4,1,4, sharex=ax1)
t_init = 0
for i in range(len(mask_array)):
    time_s = np.linspace(t_init, (len(v_tas[mask_array[i]])-1)/100 + t_init, len(v_tas[mask_array[i]]))
    ax3.plot(time_s, np.rad2deg(theta[mask_array[i]]), marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew)
    t_init = time_s[-1]+10
ax3.set_ylabel("$\\theta$ (deg)")
ax3.set_xlabel("Time $t$ (s)")
ax3.grid(which='major', linestyle='-')
ax3.grid(which='minor', linestyle=':', color='grey')
ax3.autoscale(enable=True, axis='x', tight=True)

plt.tight_layout()
plt.show()

# %%
# Removing elliptical lift CD 

def rem_ellipticalperf(CL, CD, airplane):

    CD_nell = CD - (CL**2 * (np.pi * airplane.AR)**-1)
    return CD_nell

# %%
# Getting Drag and Lift Coefficients for the correct moments in the flight

bins = np.linspace(0.05, 1.4, 50)

cl_total = np.array([])
cd_total = np.array([])
cdaoa_total = np.array([])
aoa_total = np.array([])
for i in range(len(mask_array)):
    cl_total = np.append(cl_total, CL[mask_array[i]])
    cd_total = np.append(cd_total, CD[mask_array[i]])
    cdaoa_total = np.append(cdaoa_total, CD_r2[mask_array[i]])
    aoa_total = np.append(aoa_total, alpha[mask_array[i]])
digitized = np.digitize(cl_total, bins)
cl_means = [cl_total[digitized == i].mean() for i in range(1, len(bins))]
cd_means = [cdaoa_total[digitized == i].mean() for i in range(1, len(bins))]

# Period 20
cl_total_1 = np.array([])
cd_total_1 = np.array([])
selection = np.arange(1, 8, 1)
for i in selection:
    cl_total_1 = np.append(cl_total_1, CL[mask_array[i]])
    cd_total_1 = np.append(cd_total_1, CD_r2[mask_array[i]])
digitized = np.digitize(cl_total_1, bins)
cl_means_1 = [cl_total_1[digitized == i].mean() for i in range(1, len(bins))]
cd_means_1 = [cd_total_1[digitized == i].mean() for i in range(1, len(bins))]

# Period 7.5 to 10
cl_total_2 = np.array([])
cd_total_2 = np.array([])
selection = [8, 9, 10, 11, 12, 14]
for i in selection:
    cl_total_2 = np.append(cl_total_2, CL[mask_array[i]])
    cd_total_2 = np.append(cd_total_2, CD_r2[mask_array[i]])
digitized = np.digitize(cl_total_2, bins)
cl_means_2 = [cl_total_2[digitized == i].mean() for i in range(1, len(bins))]
cd_means_2 = [cd_total_2[digitized == i].mean() for i in range(1, len(bins))]

# Period 5
cl_total_3 = np.array([])
cd_total_3 = np.array([])
selection = np.arange(15, 23, 1)
for i in selection:
    cl_total_3 = np.append(cl_total_3, CL[mask_array[i]])
    cd_total_3 = np.append(cd_total_3, CD_r2[mask_array[i]])
digitized = np.digitize(cl_total_3, bins)
cl_means_3 = [cl_total_3[digitized == i].mean() for i in range(1, len(bins))]
cd_means_3 = [cd_total_3[digitized == i].mean() for i in range(1, len(bins))]

# savemat('../Results/cl_total.mat', {'cl_total': cl_total})
# savemat('../Results/cd_total.mat', {'cd_total': cd_total})
polar_acceleration = cl_finders.cd2polar(createv, cdaoa_total, cl_total, highorder=True)
polar_acceleration_lo = cl_finders.cd2polar(createv, cd_total, cl_total, highorder=False)

polar_2021 = pd.read_pickle('../Results/2022-06-15_ct.pkl')
polar_2022_ct = np.array([polar_2021.Polar[0], polar_2021.Polar[1], polar_2021.Polar[2]])

# #polar_acceleration[1] = 0.9
print(polar_acceleration)
print(polar_acceleration_lo)

CL_fitted, CD_fitted = cl_finders.plotfittedpolar(createv, polar_acceleration, np.array([0.06, 1.4]))
CL_2022, CD_2022 = cl_finders.plotfittedpolar(createv, polar_2022_ct, np.array([0.06, 1.4]))
# plt.figure()
# plt.plot(cd_total_1, cl_total_1, marker='d', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='')
# plt.plot(cdgood_means, clgood_means, marker='s', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='')
# plt.show()

fig_cl_alpha_cl_cd = plt.figure()
ax0 = plt.subplot(1,2,1)
ax1 = plt.subplot(1,2,2)

ax0.plot(cd_means, np.array(cl_means), marker='', markeredgecolor=[0,0,0], markersize=5, markeredgewidth=markedgew, linestyle='solid', label='Averaged by CL')
ax1.errorbar(np.rad2deg(aoa_bin_means), np.array(cl_means), marker='', markeredgecolor=[0,0,0], markersize=5, markeredgewidth=markedgew, linestyle='--', label='')
plt.show()

fig_dragpolar = plt.figure(dpi=200)
plt.scatter(cdaoa_total, cl_total, c=np.rad2deg(aoa_total), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1, label='Raw Data')
plt.plot(CD_fitted, CL_fitted, marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='-', label='Curve Fit')
plt.plot(cd_means, np.array(cl_means), marker='', markeredgecolor=[0,0,0], markersize=5, markeredgewidth=markedgew, linestyle='--', label='Averaged by CL')
plt.plot(cd_means_1, np.array(cl_means_1), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 20')
plt.plot(cd_means_2, np.array(cl_means_2), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 7.5-10')
plt.plot(cd_means_3, np.array(cl_means_3), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 5')

plt.legend()
plt.colorbar(label='Angle of Attack $\\alpha$')
plt.xlabel("$C_D$")
plt.ylabel("$C_L$")
main.save_figure(fig_dragpolar, f'{fig_dragpolar=}'.split('=')[0], figure_path)
plt.show()

fig_dragpolarnell = plt.figure(dpi=200)
plt.scatter(rem_ellipticalperf(cl_total, cdaoa_total, createv), cl_total, c=np.rad2deg(aoa_total), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1, label='Raw Data')
plt.plot(rem_ellipticalperf(CL_fitted, CD_fitted, createv), CL_fitted, marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='-', label='Curve Fit')
plt.plot(rem_ellipticalperf(np.array(cl_means), cd_means, createv), np.array(cl_means), marker='', markeredgecolor=[0,0,0], markersize=5, markeredgewidth=markedgew, linestyle='--', label='Averaged by CL')
plt.plot(rem_ellipticalperf(np.array(cl_means_1), cd_means_1, createv), np.array(cl_means_1), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 20')
plt.plot(rem_ellipticalperf(np.array(cl_means_2), cd_means_2, createv), np.array(cl_means_2), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 7.5-10')
plt.plot(rem_ellipticalperf(np.array(cl_means_3), cd_means_3, createv), np.array(cl_means_3), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 5')
plt.legend()
plt.colorbar(label='Angle of Attack $\\alpha$')
plt.xlabel("$C_D - C_L^2 / \pi AR$")
plt.ylabel("$C_L$")
main.save_figure(fig_dragpolarnell, f'{fig_dragpolarnell=}'.split('=')[0], figure_path)
plt.show()

# plt.figure()
# plt.plot(cd_total, cl_total, marker=markerstyle, alpha=0.5, markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='')
# plt.plot(cdaoa_total, cl_total, marker='D', alpha=0.5, markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='')
# plt.plot(CD_fitted, CL_fitted, marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='-')
# plt.plot(cd_means, cl_means, marker=markerstyle, markeredgecolor=[0,0,0], markersize=5, markeredgewidth=markedgew, linestyle='')
# #plt.plot(cdgood_means, clgood_means, marker='s', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='')
# plt.plot(CD_2021, CL_2021, marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='-')
# plt.show()
# P_acc, EAS = cl_finders.polar2preqew(createv, polar_acceleration, (9,28), createvstandardweight=False)

fig_linearpolar = plt.figure()
plt.scatter(cl_total**2,cdaoa_total, c=np.rad2deg(aoa_total), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1, label='Raw Data')
plt.plot(CL_fitted**2, CD_fitted, marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='-', label='Curve Fit')
plt.plot(np.array(cl_means)**2, cd_means, marker='', markeredgecolor=[0,0,0], markersize=5, markeredgewidth=markedgew, linestyle='--', label='Averaged by CL')
plt.plot(np.array(cl_means_1)**2, cd_means_1, marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 20')
plt.plot(np.array(cl_means_2)**2, cd_means_2, marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 7.5-10')
plt.plot(np.array(cl_means_3)**2, cd_means_3, marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 5')
plt.legend()
plt.colorbar(label='Angle of Attack $\\alpha$')
plt.ylabel("$C_D$")
plt.xlabel("$C_L ^2$")
main.save_figure(fig_linearpolar, f'{fig_linearpolar=}'.split('=')[0], figure_path)
plt.show()

# %% [markdown]
# Plotting averages of each run, seeing if there are large differences between them.  If there are, trying to hypothesize about solutions for excluding certain data.  

# %%
import itertools

marker = itertools.cycle(('d', 'D', 's', 'o', '^', 'H', 'v', '<', '>'))
linestyles = itertools.cycle(('dotted', 'solid', 'dashed', 'dashdot'))

# Plotting AOA values
plt.figure(figsize=(10,10), dpi=150)
for i in range(len(mask_array)):
    cl_total_run = cl_finders.total_segments_boolean(CL, mask_array, np.array([i]))
    cd_total_run = cl_finders.total_segments_boolean(CD_r2, mask_array, [i])

    aoa_total_run = cl_finders.total_segments_boolean(alpha, mask_array, [i])
    P_total_run = cl_finders.total_segments_boolean(P, mask_array, [i])
    Q_total_run = cl_finders.total_segments_boolean(Q, mask_array, [i])
    theta_total_run = cl_finders.total_segments_boolean(theta, mask_array, [i])
    gamma_total_run = cl_finders.total_segments_boolean(gamma, mask_array, [i])
    h_dot_total_run = cl_finders.total_segments_boolean(Vd_tas, mask_array, [i])

    [cl_means_run, NaN, NaN, cd_means_run, NaN, NaN] = cl_finders.collect_bins(bins, cl_total_run, cd_total_run)
    plt.plot(cd_means_run, cl_means_run, linestyle=next(linestyles), marker=next(marker), label="Run #"+str(i))
    plt.scatter(cd_total_run, cl_total_run, c=np.rad2deg(aoa_total_run), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1)

plt.colorbar(label='$\\alpha$')
plt.legend()
plt.show()

# Plotting P Values
plt.figure(figsize=(10,10), dpi=150)
for i in range(len(mask_array)):
    cl_total_run = cl_finders.total_segments_boolean(CL, mask_array, np.array([i]))
    cd_total_run = cl_finders.total_segments_boolean(CD_r2, mask_array, [i])

    aoa_total_run = cl_finders.total_segments_boolean(alpha, mask_array, [i])
    P_total_run = cl_finders.total_segments_boolean(P, mask_array, [i])
    Q_total_run = cl_finders.total_segments_boolean(Q, mask_array, [i])
    theta_total_run = cl_finders.total_segments_boolean(theta, mask_array, [i])
    gamma_total_run = cl_finders.total_segments_boolean(gamma, mask_array, [i])
    h_dot_total_run = cl_finders.total_segments_boolean(Vd_tas, mask_array, [i])

    [cl_means_run, NaN, NaN, cd_means_run, NaN, NaN] = cl_finders.collect_bins(bins, cl_total_run, cd_total_run)
    plt.plot(cd_means_run, cl_means_run, linestyle=next(linestyles), marker=next(marker), label="Run #"+str(i))
    plt.scatter(cd_total_run, cl_total_run, c=P_total_run, cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1)

plt.colorbar(label='$P$')
plt.legend()
plt.show()

# Plotting Q values
plt.figure(figsize=(10,10), dpi=150)
for i in range(len(mask_array)):
    cl_total_run = cl_finders.total_segments_boolean(CL, mask_array, np.array([i]))
    cd_total_run = cl_finders.total_segments_boolean(CD_r2, mask_array, [i])

    aoa_total_run = cl_finders.total_segments_boolean(alpha, mask_array, [i])
    P_total_run = cl_finders.total_segments_boolean(P, mask_array, [i])
    Q_total_run = cl_finders.total_segments_boolean(Q, mask_array, [i])
    theta_total_run = cl_finders.total_segments_boolean(theta, mask_array, [i])
    gamma_total_run = cl_finders.total_segments_boolean(gamma, mask_array, [i])
    h_dot_total_run = cl_finders.total_segments_boolean(Vd_tas, mask_array, [i])

    [cl_means_run, NaN, NaN, cd_means_run, NaN, NaN] = cl_finders.collect_bins(bins, cl_total_run, cd_total_run)
    plt.plot(cd_means_run, cl_means_run, linestyle=next(linestyles), marker=next(marker), label="Run #"+str(i))
    plt.scatter(cd_total_run, cl_total_run, c=Q_total_run, cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1)

plt.colorbar(label='$Q$')
plt.legend()
plt.show()

# Plotting Theta values
plt.figure(figsize=(10,10), dpi=150)
for i in range(len(mask_array)):
    cl_total_run = cl_finders.total_segments_boolean(CL, mask_array, np.array([i]))
    cd_total_run = cl_finders.total_segments_boolean(CD_r2, mask_array, [i])

    aoa_total_run = cl_finders.total_segments_boolean(alpha, mask_array, [i])
    P_total_run = cl_finders.total_segments_boolean(P, mask_array, [i])
    Q_total_run = cl_finders.total_segments_boolean(Q, mask_array, [i])
    theta_total_run = cl_finders.total_segments_boolean(theta, mask_array, [i])
    gamma_total_run = cl_finders.total_segments_boolean(gamma, mask_array, [i])
    h_dot_total_run = cl_finders.total_segments_boolean(Vd_tas, mask_array, [i])

    [cl_means_run, NaN, NaN, cd_means_run, NaN, NaN] = cl_finders.collect_bins(bins, cl_total_run, cd_total_run)
    plt.plot(cd_means_run, cl_means_run, linestyle=next(linestyles), marker=next(marker), label="Run #"+str(i))
    plt.scatter(cd_total_run, cl_total_run, c=np.rad2deg(theta_total_run), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1)

plt.colorbar(label='$\theta$')
plt.legend()
plt.show()

# Plotting gamma values
plt.figure(figsize=(10,10), dpi=150)
for i in range(len(mask_array)):
    cl_total_run = cl_finders.total_segments_boolean(CL, mask_array, np.array([i]))
    cd_total_run = cl_finders.total_segments_boolean(CD_r2, mask_array, [i])

    aoa_total_run = cl_finders.total_segments_boolean(alpha, mask_array, [i])
    P_total_run = cl_finders.total_segments_boolean(P, mask_array, [i])
    Q_total_run = cl_finders.total_segments_boolean(Q, mask_array, [i])
    theta_total_run = cl_finders.total_segments_boolean(theta, mask_array, [i])
    gamma_total_run = cl_finders.total_segments_boolean(gamma, mask_array, [i])
    h_dot_total_run = cl_finders.total_segments_boolean(Vd_tas, mask_array, [i])

    [cl_means_run, NaN, NaN, cd_means_run, NaN, NaN] = cl_finders.collect_bins(bins, cl_total_run, cd_total_run)
    plt.plot(cd_means_run, cl_means_run, linestyle=next(linestyles), marker=next(marker), label="Run #"+str(i))
    plt.scatter(cd_total_run, cl_total_run, c=np.rad2deg(gamma_total_run), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1)

plt.colorbar(label='$\gamma$')
plt.legend()
plt.show()


fig_dragpolar = plt.figure(dpi=200)
plt.scatter(cdaoa_total, cl_total, c=np.rad2deg(aoa_total), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1, label='Raw Data')
plt.plot(CD_fitted, CL_fitted, marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='-', label='Curve Fit')
plt.plot(cd_means, np.array(cl_means), marker='', markeredgecolor=[0,0,0], markersize=5, markeredgewidth=markedgew, linestyle='--', label='Averaged by CL')
plt.plot(cd_means_1, np.array(cl_means_1), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 20')
plt.plot(cd_means_2, np.array(cl_means_2), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 7.5-10')
plt.plot(cd_means_3, np.array(cl_means_3), marker='', markeredgecolor=[0,0,0], markersize=3, markeredgewidth=markedgew, linestyle='--', label='Period 5')

plt.legend()
plt.colorbar(label='Angle of Attack $\\alpha$')
plt.xlabel("$C_D$")
plt.ylabel("$C_L$")
plt.show()

# %%
fig_cl_alpha = plt.figure()
plt.scatter(np.rad2deg(aoa_total), cl_total, c=np.rad2deg(aoa_total), cmap='coolwarm', marker='D', alpha=0.5, edgecolors=[0,0,0], s=2, linewidths=0.1, label='Raw Data')
plt.plot(np.rad2deg(aoa_bin_means), cl_means, marker='', linestyle='solid')
plt.show()

# %%
# Overplotting the results of the 6 Accelerations #

P_acc, EAS = cl_finders.polar2preqew(createv, polar_acceleration, (8,28), createvstandardweight=False)

markerstyle = 'o'
markerevery = 1
markedgew = 0.1
plt.figure(figsize=(14,7))
ax1 = plt.subplot(1,2,1)
for i in range(len(mask_array)):
    ax1.plot(v_tas[mask_array[i]], P_req_r2[mask_array[i]], marker='.', markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='', alpha=0.2)
#ax1.plot(V_xflr, P_xflr, marker=markerstyle, markeredgecolor=[0,0,0], markersize=4, markevery=markerevery, markeredgewidth=markedgew, linestyle='-', label='XFLR Bare Wing')
ax1.plot(EAS, P_acc, marker=markerstyle, markeredgecolor=[0,0,0], markersize=4, markevery=markerevery, markeredgewidth=markedgew, linestyle='-', label='Accelerated Test Fitted')
ax1.set_ylabel("Power Required (W)")
ax1.set_xlabel("True Airspeed (m/s)")
ax1.grid(which='major', linestyle='-')
ax1.grid(which='minor', linestyle=':', color='grey')
ax1.legend()
ax1.autoscale(enable=True, axis='x', tight=True)

ax2 = plt.subplot(1,2,2)
for i in range(len(mask_array)):
    ax2.plot(v_tas[mask_array[i]], D[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='')
ax2.set_ylabel("Drag (N)")
ax2.set_xlabel("True Airspeed (m/s)")
ax2.grid(which='major', linestyle='-')
ax2.grid(which='minor', linestyle=':', color='grey')
ax2.autoscale(enable=True, axis='x', tight=True)
plt.tight_layout()
plt.show()

# %%
# Plotting all terms of power required to see significance

markerstyle = 'o'
markerevery = 1
markedgew = 0.1
plt.figure(figsize=(14,7))
ax1 = plt.subplot(3,1,1)
for i in range(len(mask_array)):
    ax1.plot(v_tas[mask_array[i]], P_req_t1[mask_array[i]]/P_req[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='')
ax1.set_ylabel("Thrust Power (W)")
ax1.set_xlabel("True Airspeed (m/s)")
ax1.grid(which='major', linestyle='-')
ax1.grid(which='minor', linestyle=':', color='grey')
ax1.autoscale(enable=True, axis='x', tight=True)

ax2 = plt.subplot(3,1,2,sharex=ax1)
for i in range(len(mask_array)):
    ax2.plot(v_tas[mask_array[i]], P_req_t2[mask_array[i]]/P_req[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='')
ax2.set_ylabel("Climb Power (W)")
ax2.set_xlabel("True Airspeed (m/s)")
ax2.grid(which='major', linestyle='-')
ax2.grid(which='minor', linestyle=':', color='grey')
ax2.autoscale(enable=True, axis='x', tight=True)

ax3 = plt.subplot(3,1,3, sharex=ax1)
for i in range(len(mask_array)):
    ax3.plot(v_tas[mask_array[i]], P_req_t3[mask_array[i]]/P_req[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='')
ax3.set_ylabel("Acceleration Power (W)")
ax3.set_xlabel("True Airspeed (m/s)")
ax3.grid(which='major', linestyle='-')
ax3.grid(which='minor', linestyle=':', color='grey')
ax3.autoscale(enable=True, axis='x', tight=True)

# %%
# Plotting the drag polar of the Vehicle

markerstyle = 'o'
markerevery = 1
markedgew = 0.1
plt.figure(figsize=(14,7))
ax1 = plt.subplot(1,2,1)
for i in range(len(mask_array)):
    ax1.plot(CD_r2[mask_array[i]], CL[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='')
ax1.set_ylabel("Lift Coefficient")
ax1.set_xlabel("Drag Coefficient")
ax1.grid(which='major', linestyle='-')
ax1.grid(which='minor', linestyle=':', color='grey')
ax1.autoscale(enable=True, axis='x', tight=True)

ax2 = plt.subplot(1,2,2)
for i in range(len(mask_array)):
    ax2.plot(v_tas[mask_array[i]], D[mask_array[i]], marker=markerstyle, markeredgecolor=[0,0,0], markersize=3, markevery=markerevery, markeredgewidth=markedgew, linestyle='')
ax2.set_ylabel("Drag (N)")
ax2.set_xlabel("True Airspeed (m/s)")
ax2.grid(which='major', linestyle='-')
ax2.grid(which='minor', linestyle=':', color='grey')
ax2.autoscale(enable=True, axis='x', tight=True)
plt.tight_layout()
plt.show()

# %%
# Standard Data Exporting and Prep

cl_total_stand = cl_finders.total_segments_boolean(CL, mask_array)
cd_total_stand = cl_finders.total_segments_boolean(CD_r2, mask_array)

# Taking totals and binning
bins = np.linspace(0.2, 1.4, 51)
[cl_means, cl_stds, cl_ci95s, cd_means, cd_stds, cd_ci95s] = cl_finders.collect_bins(bins, cl_total_stand, cd_total_stand)

# Fitting the results
polar_acc = cl_finders.cd2polar(createv, cd_means, cl_means, highorder=True)

# Packaging the results
acceleration_binresult = cl_finders.packaging_binresults(cl_total_stand, cl_means, cl_stds, cl_ci95s, cd_total_stand, cd_means, cd_stds, cd_ci95s, polar_acc, createv)

# %% [markdown]
# ## Exporting Results

# %%
# Exporting workspace
import dill
location = result_path + 'workspace.pkl'
dill.dump_session(location)

# Exporting polar data
pd.to_pickle(acceleration_binresult, result_path+'acceleration_binresult.pkl')


