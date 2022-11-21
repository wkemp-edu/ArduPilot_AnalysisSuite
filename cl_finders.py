# Find the coefficient of lift for all points in data vector

import numpy as np
from scipy.optimize import curve_fit
import fittingtools

rho_ssl = 1.225

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

    CL = (aircraft.weight / np.cos(phi)) - (aircraft.mass * W_dot)

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
  

def cd2polar(aircraft, CD, CL):
    # Turning individual data points of CD and CL into a drag polar
    # Inputs: 
        # 1. Aircraft parameters
        # 2. Drag coefficient data points
        # 3. Lift coefficient data points
    # Outputs: 
        # 1. Oswald efficiency
        # 2. Zero lift drag coefficient
    
    popt, pcov = curve_fit(polarcurve_fit, CL**2, CD)
    CD0 = popt[0]
    K = popt[1]
    e = (np.pi * aircraft.AR * K)**-1
    return np.array([CD0, e])

def polar2preqew(aircraft, polar, airspeed_range):
    # Turning the fitted drag polar parameters into a power required for steady level flight @ standard SL
    # Inputs:
        # 1. Polar:  array of numpy (size determines the polar equation used)
        # 2. Airspeed range, range of airspeeds desired for analysis (tuple)
        
    # Outputs:
        # 1. Power required (W) @ SL @ standard weight (12.6 kg)
        # 2. Equivalent Airspeed @ Standard sea level
        
    rho_ssl = 1.225 # Standard density at sea level
    mass_ew = 12.6 # The standard mass of the aircraft
    W_ew = 12.6 * 9.807
    EAS_SL = np.linspace(airspeed_range[0], airspeed_range[1], 500)
    
    if len(polar) == 2:
        Pew_req = ((0.5 * rho_ssl * EAS_SL**3) * aircraft.area * polar[0]) + (2 * W_ew**2 * (np.pi * rho_ssl * EAS_SL * aircraft.span**2 * polar[1])**-1)
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
    CD = polar[0] + ((np.pi*aircraft.AR*polar[1])**-1 * CL**2)
    if len(CL_alpha) == 3:
        AoA = CL_alpha[0]*CL**2 + CL_alpha[1]*CL + CL_alpha[2]
    elif len(CL_alpha) == 2:
        AoA = CL_alpha[0]*CL**1 + CL_alpha[1]
    
    return V_eas_ssl, CL, CD, AoA