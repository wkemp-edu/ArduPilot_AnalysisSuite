# Find the coefficient of lift for all points in data vector

import numpy as np
from scipy.optimize import curve_fit
import fittingtools
import pandas as pd
import main

rho_ssl = 1.225

class data():
    def __init__(self, prop, motor, aircraft):
        self.prop = prop
        self.motor = motor
        self.aircraft = aircraft
        return
    
    def add_cruise(self, df):
        # Attitude 
        self.phi = np.deg2rad(df["RollAngle"].to_numpy())            # Bank angle in radians
        self.pitch = np.deg2rad(df["PitchAngle"].to_numpy())         # Pitch angle in radians

        # Inertial Measurement Unit
        self.U_dot = df["XAcc_IMU"].to_numpy()                       # Acceleration in X direction

        # Atmospheric adjustments:
        self.rho = df["Pressure"].to_numpy() * (287 * (df["Ambient_Temperature"].to_numpy()+273.15))**-1             # Density found from barometer pressure & airspeed sensor temperatures
        self.v_eas = df["Airspeed_Sensor0"].to_numpy()                               # Equivalent SSL airspeed (m/s)
        self.v_tas = self.v_eas * np.sqrt(1.225) * np.sqrt(self.rho)**-1   # the true airspeed
        self.q = 0.5 * self.rho * self.v_tas**2                            # Dynamic pressure 

        # For Descent method
        self.h = df["Altitude_POS"].to_numpy()                   # Altitude
        self.Vd_eas = df["DescendingXK"].to_numpy()                  # Descent Rate from EKF (is it true or EAS at SSL?)
        self.Vd_tas = self.Vd_eas * np.sqrt(1.225) * np.sqrt(self.rho)**-1   # the true airspeed

        # Ground speed limiter
        self.v_dem = df["Airspeed_Demanded"].to_numpy()

        # Propulsion characterization
        self.n = df["MotorRPM"].to_numpy() / 60                               # Revolutions per second
        self.i_esc = df["EscCurrent"].to_numpy()
        self.v_esc = df["EscVoltage"].to_numpy()
        self.J = self.v_tas / (self.n * self.prop.diameter)
        self.eff = self.prop.efficiency(self.J) * self.motor.efficiency(self.n, self.i_esc)

        # Estimated propulsive power (New Fitting)
        self.P_eta = eta_steady(self.prop, self.motor, self.v_tas, self.n, self.i_esc, self.v_esc)
        self.P_ct = thrust_steady(self.prop, self.rho, self.v_tas, self.n)

        # Getting drag coefficient
        self.Cd_eta = preq2cd(self.aircraft, self.v_tas, self.q, self.P_eta)
        self.Cd_ct = preq2cd(self.aircraft, self.v_tas, self.q, self.P_ct)

        # Getting lift coefficient
        self.CL = cl_banked(self.aircraft, self.q, self.phi)
        return
    
    def add_descent(self, df):

        # Attitude 
        self.phi = np.deg2rad(df["RollAngle"].to_numpy())            # Bank angle in radians
        self.pitch = np.deg2rad(df["PitchAngle"].to_numpy())         # Pitch angle in radians

        # Inertial Measurement Unit
        self.U_dot = df["XAcc_IMU"].to_numpy()                       # Acceleration in X direction

        # Atmospheric adjustments:
        self.rho = df["Pressure"].to_numpy() * (287 * (df["Ambient_Temperature"].to_numpy()+273.15))**-1             # Density found from barometer pressure & airspeed sensor temperatures
        self.v_eas = df["Airspeed_Sensor0"].to_numpy()                               # Equivalent SSL airspeed (m/s)
        self.v_tas = self.v_eas * np.sqrt(1.225) * np.sqrt(self.rho)**-1   # the true airspeed
        self.q = 0.5 * self.rho * self.v_tas**2                            # Dynamic pressure 

        # For Descent method
        self.h = df["Altitude_POS"].to_numpy()                   # Altitude
        self.Vd_eas = df["DescendingXK"].to_numpy()                  # Descent Rate from EKF (is it true or EAS at SSL?)
        self.Vd_tas = self.Vd_eas * np.sqrt(1.225) * np.sqrt(self.rho)**-1   # the true airspeed

        # Ground speed limiter
        self.v_dem = df["Airspeed_Demanded"].to_numpy()

        # Propulsion characterization
        self.n = df["MotorRPM"].to_numpy() / 60                               # Revolutions per second
        self.i_esc = df["EscCurrent"].to_numpy()
        self.v_esc = df["EscVoltage"].to_numpy()
        self.J = self.v_tas / (self.n * self.prop.diameter)

        self.P_desc = desc2preq(self.aircraft, self.prop, self.rho, self.v_tas, self.n, self.q, self.Vd_tas)

        # Getting drag coefficient
        self.Cd_desc = preq2cd(self.aircraft, self.v_tas, self.q, self.P_desc)

        # Getting lift coefficient
        self.CL = cl_banked(self.aircraft, self.q, self.phi)

# def masker_lua(throttle, current, dataframe):
#     # Basic mask for Lua accelerations
#     # Checking for throttle to zero, and current above 5A

#     bin_mask = (dataframe['ThrottleOut'] == 1900) & (dataframe['MainBatteryCurrent'] >= 5)
#     jump = np.diff(bin_mask)
#     start_mask = jump > 0
#     end_mask = jump < 0
#     start_indices = np.where(start_mask)
#     end_indices = np.where(end_mask)

#     start_times = dataframe.index(start_indices)
#     end_times = dataframe.index(end_indices)

def cl_banked(aircraft, q, phi):
    
    # Find CL estimate from:
        # 1. Aircraft weight (m*g)
        # 2. Bank angle (phi) in radians
        # 3. Dynamic pressure (the actual one)
        
    # Return CL estimate
    
    CL = aircraft.weight * (np.cos(phi) * q * aircraft.area)**-1
    return CL

def cl_usbanked(aircraft, q, phi, W_dot):

    # Find CL estimate from:
        # 1. Aircraft weight (m*g)
        # 2. Bank angle
        # 3. Dynamic pressure
        # 4. Acceleration of CM relative to Earth, taken in the body frame, expressed in frd

    CL = ((aircraft.weight / np.cos(phi)) - (aircraft.mass * W_dot)) * (q * aircraft.area)**-1
    return CL

# def cl_accelerated(aircraft, acc_x, acc_y, acc_z, p, q, r, phi, theta, psi, aoa):
    
#     # Find CL estimate from:
#         # 1. Actual acceleration of aircraft orthogonal to wind axis
#             # a) IMU acceleration (Expressed in frame of IMU)
#             # b) Rotational rates (Compensates for IMU not at CG)
#             # c) AoA for rotating the acceleration into the aerodynamic frame
#             # Result is a_wind_z 
            
#     CL = a_wind_z * aircraft.mass

def eta_steady(propeller, motor, v_tas, n, current, voltage, oldfit=False):
    # Estimated propulsive power
    J_tas = v_tas * (n * propeller.diameter)**-1             # Advance ratio at TAS
    P_motor = current * voltage                              # Electrical power to ESC

    eta_motor = motor.efficiency(n, current)                 # Filtering out inf due to n=0
    eta_prop = propeller.efficiency(J_tas)                   # System efficiency including Zubax ESC
    eta_sys = eta_motor * eta_prop
    P_eta = P_motor * eta_sys                                # Propulsive power estimate from efficiency
    return P_eta

def thrust_steady(propeller, rho, v_tas, n, oldfit=False):
    J_tas = v_tas * (n * propeller.diameter)**-1             # Advance ratio at TAS
    ct = propeller.thrust_coeff(J_tas)                  # Thrust coefficients
    T = ct * propeller.diameter**4 * rho * n**2              # Getting Thrust from thrust coefficient
    P_ct = v_tas * T                                    # Propulsive power estimate from thrust coefficient
    return P_ct

def cruise_highorder(aircraft, propeller, rho, v_tas, n, Vd, theta, U_dot):
    # Produce power required, with acceleration and potential terms
    # Inputs:
        # 1. aircraft
        # 2. v_tas in true airspeed
        # 4. Theta pitch angle
        # 5. Acceleration in X body frame U dot
    # Outputs:
        # 1. Power required (watts)
    J_tas = v_tas * (n * propeller.diameter)**-1
    ct = propeller.thrust_coeff(J_tas)
    T = ct * rho * n**2 * propeller.diameter**4

    D = T + (aircraft.weight * Vd) - (aircraft.mass * U_dot)
    P_req = D * v_tas
    return P_req

def desc2preq(aircraft, propeller, rho, v, n, q, Vd, oldfit=False):
    # Inputs:
        # Aircraft
        # v in true airspeed
        # q dynamic pressure
        # Vd descent rate (true)
    W = aircraft.weight 
    P_req = W * Vd          # Total power required with propeller drag included

    D_prop = np.abs(propeller.freewheel_tcoeff()) * rho * n**2 * propeller.diameter**4
    P_propdrag = D_prop * v

    print("Propeller Drag" + str(P_propdrag))
    P_req = P_req - P_propdrag  # Removing the drag from the freewheeling propeller
    return P_req

def descU2preq(aircraft, propeller, rho, v, n, q, Vd, theta, U_dot):
    # Inputs:
        # Aircraft
        # v in true airspeed
        # q dynamic pressure
        # Vd descent rate (true)
    D = (aircraft.weight * Vd) - (aircraft.mass * U_dot)
    P_req = D * v
    D_prop = np.abs(propeller.freewheel_tcoeff()) * rho * n**2 * propeller.diameter**4
    P_propdrag = D_prop * v

    print("Propeller Drag" + str(P_propdrag))
    P_req = P_req - P_propdrag  # Removing the drag from the freewheeling propeller
    return P_req

# The function that should be used for gliding power required measurements
def desc2preq_WT(aircraft, propeller, rho, v, Vd):
    # Inputs:
        # Aircraft
        # v in true airspeed
        # q dynamic pressure
        # Vd descent rate (true)
    RPM_wtfit = propeller.getFreewheelRPM(v)
    n_wtfit = RPM_wtfit / 60 
    J_wtfit = v / (n_wtfit * propeller.diameter)

    CT_wtfit = propeller.thrust_coeff(J_wtfit)
    D_wtfit = -1*(CT_wtfit) * rho * n_wtfit**2 * propeller.diameter**4
    D_total = (aircraft.weight * Vd) / v
    P_req = (D_total - D_wtfit) * v

    print("Propeller Drag" + str(D_wtfit))
    return P_req
    
def preq2cd(aircraft, v, q, p_required):
    # Inputs:
        # Aircraft --> wing area
        # v in true airspeed
        # q dynamic pressure
        # p_required from method
    
    CD = p_required * (q * v * aircraft.area)**-1
    return CD

def polarcurve_fit(CL_sq, cd0, k):
    # of the form CD = CD0 + kCL^2
    return cd0 + k * CL_sq

def polarcurve_fit_ho(C_L, cd0, k, C_Lmind):
    # Of the form CD = CD0 + k(C_L - C_Lmind)^2
    return cd0 + k * (C_L - C_Lmind)**2
  
def cd2polar(aircraft, CD, CL, highorder=False):
    # Turning individual data points of CD and CL into a drag polar
    # Inputs: 
        # 1. Aircraft parameters
        # 2. Drag coefficient data points
        # 3. Lift coefficient data points
    # Outputs: 
        # 1. Oswald efficiency
        # 2. Zero lift drag coefficient
    
    if highorder:
        popt, pcov = curve_fit(polarcurve_fit_ho, CL, CD, p0=[0.02, 0.05, 0.5], maxfev=50000, method='dogbox', bounds=((0, 0, 0), (100.0, 100.0, 100.0)))
        CD0 = popt[0]
        K = popt[1]
        CL_mind = popt[2]
        e = (np.pi * aircraft.AR * K)**-1

        # Printing Fit Quality (R^2)
        residuals = CD - polarcurve_fit_ho(CL, *popt)
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((CD - np.mean(CD))**2)
        r_square = 1 - (ss_res/ss_tot)
        print(r_square)
        return np.array([CD0, e, CL_mind])
    else:
        popt, pcov = curve_fit(polarcurve_fit, CL**2, CD, p0=[0, 0.01], maxfev=50000, method='dogbox')
        CD0 = popt[0]
        K = popt[1]
        e = (np.pi * aircraft.AR * K)**-1

        # Printing Fit Quality (R^2)
        residuals = CD - polarcurve_fit(CL, *popt)
        ss_res = np.sum(residuals**2)
        ss_tot = np.sum((CD - np.mean(CD))**2)
        r_square = 1 - (ss_res/ss_tot)
        print(r_square)
        return np.array([CD0, e])

def plotfittedpolar(aircraft, polar, CL_range):

    # Plotting the fitted drag polar over the desired lift range

    CL_vector = np.linspace(CL_range[0], CL_range[1], 200)

    if len(polar) == 2:
        CD_vector = polar[0] + (polar[1]*np.pi*aircraft.AR)**-1 * CL_vector**2
    elif len(polar) == 3:
        CD_vector = polar[0] + (polar[1]*np.pi*aircraft.AR)**-1 * (CL_vector - polar[2])**2

    return CL_vector, CD_vector

def polar2preqew(aircraft, polar, airspeed_range, createvstandardweight=True):
    # Turning the fitted drag polar parameters into a power required for steady level flight @ standard SL
    # Inputs:
        # 1. Polar:  array of numpy (size determines the polar equation used)
        # 2. Airspeed range, range of airspeeds desired for analysis (tuple)
        
    # Outputs:
        # 1. Power required (W) @ SL @ standard weight (12.6 kg)
        # 2. Equivalent Airspeed @ Standard sea level
    
    rho_ssl = 1.225 # Standard density at sea level

    if createvstandardweight == True:
        mass_ew = 12.6 # The standard mass of the aircraft
        W_ew = 12.6 * 9.807
    else:
        W_ew = aircraft.weight
    
    EAS_SL = np.linspace(airspeed_range[0], airspeed_range[1], 500)
    
    if len(polar) == 2:
        Pew_req = ((0.5 * rho_ssl * EAS_SL**3) * aircraft.area * polar[0]) + (2 * W_ew**2 * (np.pi * rho_ssl * EAS_SL * aircraft.span**2 * polar[1])**-1)
    elif len(polar) == 3:
        Pew_req = ((0.5 * rho_ssl * EAS_SL**3) * aircraft.area * polar[0]) + ( (0.5*rho_ssl*EAS_SL**3*aircraft.area) * ((np.pi*aircraft.AR*polar[1])**-1) * ((2*W_ew*(rho_ssl*EAS_SL**2*aircraft.area)**-1) - polar[2])**2)
    return Pew_req, EAS_SL

def rawpolar2preqew(aircraft, CL, CD):
    # Turning the fitted drag polar parameters into a power required for steady level flight @ standard SL
    # Inputs:
        # 1. Polar:  array of numpy (size determines the polar equation used)
        # 2. Airspeed range, range of airspeeds desired for analysis (tuple)
        
    # Outputs:
        # 1. Power required (W) @ SL @ standard weight (12.6 kg)
        # 2. Equivalent Airspeed @ Standard sea level
    
    rho_ssl = 1.225 # Standard density at sea level
    W_ew = aircraft.weight
    
    EAS_SL = np.sqrt(W_ew / (0.5 * rho_ssl * aircraft.area * CL))
    Pew_req = CD * 0.5 * rho_ssl * EAS_SL**3 * aircraft.area
    return Pew_req, EAS_SL

def angleModel(CL, aoa, model):
    # Fitting CL to angle of attack using 2nd order polynomial
    # Inputs:
        # 1. Estimated Lift Coefficient
        # 2. Estimated Angle of Attack (radians) converted to degrees for fit!
    # Outputs:
        # 1. CL_alpha -> numpy array containing coefficients (a, b, c) where alpha(CL) = a*CL^2 + b*CL^1 + c
    
    aoa = np.rad2deg(aoa)

    if model == "linear":
        popt, pcov = curve_fit(fittingtools.fit_CLa_lin, CL, aoa)
        
    
    elif model == "quadratic":
        popt, pcov = curve_fit(fittingtools.fit_CLa, CL, aoa)

    CL_alpha = popt
    return CL_alpha

def basicModel(aircraft, polar, CL_alpha, airspeed_range):
    # Output for solar model
    # Inputs:
        # 1. Polar:  array of numpy (size determines the polar equation used)
        # 2. Airspeed range, range of airspeeds desired for analysis (tuple)
    # Outputs:
        # 1. Coefficient of Lift
        # 2. Coefficient of Drag
        # 3. Angle of attack

    # ** Need to call CL_alpha fit beforehand!

    V_eas_ssl = np.linspace(airspeed_range[0], airspeed_range[1], 500)
    CL = (aircraft.weight) / (0.5 * rho_ssl * V_eas_ssl**2 * aircraft.area)
    
    if len(polar) == 3:
        CD = polar[0] + ((np.pi*aircraft.AR*polar[1])**-1 * (CL - polar[2])**2)
    elif len(polar) == 2:
        CD = polar[0] + ((np.pi*aircraft.AR*polar[1])**-1 * CL**2)

    if len(CL_alpha) == 3:
        AoA = CL_alpha[0]*CL**2 + CL_alpha[1]*CL + CL_alpha[2]
    elif len(CL_alpha) == 2:
        AoA = CL_alpha[0]*CL**1 + CL_alpha[1]
    
    return V_eas_ssl, CL, CD, AoA

def mask_fromTime(df, start_time, end_time):
    # Getting boolean mask from start and end times, using full date picked out

    mask = (df.index > start_time) & (df.index < end_time)
    return mask



def get_datetime(hour_string, year, month, day):
    # Results completed datetime from hour string, and date
    split_nums = hour_string.split(':')
    hours = int(split_nums[0])
    minutes = int(split_nums[1])
    seconds = int(split_nums[2])
    return pd.Timestamp(year=year, month=month, day=day, hour=hours, minute=minutes, second=seconds)

def total_segments(variable, mask_startend):
    total = np.array([])
    for i in range(len(mask_startend)):
        total = np.append(total, variable[mask_startend[i,0]:mask_startend[i,1]])
    return total

def total_segments_boolean(variable, mask_array, selection=None):
    total = np.array([])
    if selection != None:
        for i in selection:
            total = np.append(total, variable[mask_array[i]])
    else:
        for i in range(len(mask_array)):
            total = np.append(total, variable[mask_array[i]])
    return total

def collect_bins(bins, total_lift_coeffs, total_drag_coeffs):
    # Inputs:
    # 1. bins: Numpy array that defines the bounds of the start and ends of each Coefficient of lift bin
    # 2. lift_coeffs: Numpy array containing all usable lift raw data points (Time indexed)
    # 3. drag_coeffs: Numpy array containing all usable drag raw data points (Time indexed)

    # Outputs:
    # 1. means: Numpy array containing the means of all data points that lie within each defined bin
    # 2. stds: Numpy array containing the standard deviations of all data points that lie within each defined bin
    # 3. ci95s: Numpy array containing the 95% confident intervals of all data points that lie within each defined bin
    
    digitized = np.digitize(total_lift_coeffs, bins)      # Creating indexed binning array

    # Using indexed binning array to generate means, standard deviations, and confidence intervals
    cl_means = [total_lift_coeffs[digitized == i].mean() for i in range(1, len(bins))]
    cl_stds = [total_lift_coeffs[digitized == i].std() for i in range(1, len(bins))]
    cl_ci95s = [ 1.96 * (np.sqrt(len(total_lift_coeffs[digitized == i])))**-1 * total_lift_coeffs[digitized == i].std() for i in range(1, len(bins)) ]
    # cl_ci95s = [ 1.96 * (np.sqrt(len(digitized == i)))**-1 * total_lift_coeffs[digitized == i].std() for i in range(1, len(bins)) ] original definition

    cd_means = [total_drag_coeffs[digitized == i].mean() for i in range(1, len(bins))]
    cd_stds = [total_drag_coeffs[digitized == i].std() for i in range(1, len(bins))]
    cd_ci95s = [ 1.96 * (np.sqrt(len(total_drag_coeffs[digitized == i])))**-1 * total_drag_coeffs[digitized == i].std() for i in range(1, len(bins)) ]

    cl_means = remove_nan(cl_means)
    cl_stds = remove_nan(cl_stds)
    cl_ci95s = remove_nan(cl_ci95s)
    cd_means = remove_nan(cd_means)
    cd_stds = remove_nan(cd_stds)
    cd_ci95s = remove_nan(cd_ci95s)

    return [cl_means, cl_stds, cl_ci95s, cd_means, cd_stds, cd_ci95s]

def remove_nan(variable):
    variable = np.array(variable)
    variable = variable[~np.isnan(variable)]
    return variable

def collect_segments(mask_array, lift_coeffs, drag_coeffs):
    # Inputs:
    # 1. mask_array: Numpy array containing boolean arrays for selecting each segment
    # 2. lift_coeffs: Numpy array containing all usable lift raw data points (Time indexed)
    # 3. drag_coeffs: Numpy array containing all usable drag raw data points (Time indexed)

    # Outputs:
    # 1. means: Numpy array containing the means of all data points that lie within each defined segment
    # 2. stds: Numpy array containing the standard deviations of all data points that lie within each defined segment
    # 3. ci95s: Numpy array containing the 95% confident intervals of all data points that lie within each defined segment

    cd_means = np.zeros(len(mask_array))
    cd_stds = np.zeros(len(mask_array))
    cd_ci95s = np.zeros(len(mask_array))

    cl_means = np.zeros(len(mask_array))
    cl_stds = np.zeros(len(mask_array))
    cl_ci95s = np.zeros(len(mask_array))

    if np.shape(mask_array)[1]==len(lift_coeffs):
        for i in range(np.shape(mask_array)[0]):
            cd_means[i] = np.mean(drag_coeffs[mask_array[i]])
            cd_stds[i] = np.std(drag_coeffs[mask_array[i]])
            cd_ci95s[i] = 1.96 * cd_stds[i] * (np.sqrt(len(drag_coeffs[mask_array[i]])) **-1)

            cl_means[i] = np.mean(lift_coeffs[mask_array[i]])
            cl_stds[i] = np.std(lift_coeffs[mask_array[i]])
            cl_ci95s[i] = 1.96 * cl_stds[i] * (np.sqrt(len(lift_coeffs[mask_array[i]])) **-1)
    else:
        for i in range(len(mask_array)):
            cd_means[i] = np.mean(drag_coeffs[int(mask_array[i, 0]):int(mask_array[i, 1])])
            cd_stds[i] = np.std(drag_coeffs[int(mask_array[i, 0]):int(mask_array[i, 1])])
            cd_ci95s[i] = 1.96 * cd_stds[i] * (np.sqrt(len(drag_coeffs[int(mask_array[i, 0]):int(mask_array[i, 1])])) **-1)

            cl_means[i] = np.mean(lift_coeffs[int(mask_array[i, 0]):int(mask_array[i, 1])])
            cl_stds[i] = np.std(lift_coeffs[int(mask_array[i, 0]):int(mask_array[i, 1])])
            cl_ci95s[i] = 1.96 * cl_stds[i] * (np.sqrt(len(lift_coeffs[int(mask_array[i, 0]):int(mask_array[i, 1])])) **-1)
    return [cl_means, cl_stds, cl_ci95s, cd_means, cd_stds, cd_ci95s]

def get_mask(df, start, end, year, month, day):
    # Getting boolean mask from start and end times
    start_time = get_datetime(start, year, month, day)
    end_time = get_datetime(end, year, month, day)
    mask = (df.index > start_time) & (df.index < end_time)
    return mask

def get_maskarray(df, segment_times, year, month, day):
    masks = []
    for i in range(np.shape(segment_times)[0]):
        mask = get_mask(df, segment_times[i,0], segment_times[i,1], year, month, day)
        masks.append(mask)
    return masks

def get_datetime(hour_string, year, month, day):
    # Results completed datetime from hour string, and date
    split_nums = hour_string.split(':')
    hours = int(split_nums[0])
    minutes = int(split_nums[1])
    seconds = int(split_nums[2])
    return pd.Timestamp(year=year, month=month, day=day, hour=hours, minute=minutes, second=seconds)

# Removing elliptical lift CD 
def rem_ellipticalperf(CL, CD, airplane):

    CD_nell = CD - (CL**2 * (np.pi * airplane.AR)**-1)
    return CD_nell

# Work in progress
def packaging_binresults(cl_total, cl_means, cl_stds, cl_ci95s, cd_total, cd_means, cd_stds, cd_ci95s, polarfit, aircraft):

    # Packaging raw polars
    rawpolar = pd.DataFrame.from_dict({'CD': cd_total, 'CL': cl_total})
    # Packaging averaged polars
    avepolar = pd.DataFrame.from_dict({'CD': cd_means, 'CL': cl_means})
    # Packaging standard deviation polars
    stdpolar = pd.DataFrame.from_dict({'CD': cd_stds, 'CL': cl_stds})
    # Packaging 95% CI polars
    ci95polar = pd.DataFrame.from_dict({'CD': cd_ci95s, 'CL': cl_ci95s})

    package = main.result(rawpolar, avepolar, stdpolar, ci95polar, polarfit, aircraft)
    return package