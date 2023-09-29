from os import sys
import os
sys.path.append('../../')

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from os import sys
import os
sys.path.append('../')

# Getting packages #

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from scipy import optimize 
from scipy.io import savemat

import plotly.express as px

import main
import propellers
import motors
from aircraft import airplane
import cl_finders

import data_exporter
import datetime

class analysis():
    def __init__(self, investigation_name, data_folder, file_name):
        # Inputs:
        # 1. name: name of the analysis, how the results/figures folders will be named as well!
        # 2. data_folder: Folder containing data, under "data" directory
        # 3. file_name: Name of the file to import (BIN files)

        # Setting up and storing strings for folder paths
        self.data_folder = data_folder      # Folder containing data, under "data" directory
        self.file_name = file_name          # Name of the file to import (BIN files)

        self.result_path = '../Results/'+investigation_name+'/'
        self.figure_path = '../Figures/'+investigation_name+'/'

        self.setup_folders()            # Creating folders if not already there
        self.setup_plotstyle()          # Setting plot defaults

    def select_segments(self, segment_times=None, year=None, month=None, day=None):
        # Generates an array of segment start/end times, and a list of boolean masks for each segment.  
        
        # either input an numpy array for interval selection, then generate the masks to store internally. Or start the graphical selection tool to start selecting parts.  
        # Check the results folder for a mask already there, use this if it is present. 

        if segment_times is None:
            # Starting selection process using GUI
            self.masks = []
        
        else:
            # Using the input segment times to generate a mask
            self.segment_times = segment_times          # Saving the segment time array for storage alongside the generated masks
            self.masks = []                             # List of boolean masks that are stored for each segment (Need to generate)
            for i in range(np.shape(segment_times)[0]):
                mask = cl_finders.get_mask(self.df, segment_times[i,0], segment_times[i,1], year, month, day)
                self.masks.append(mask)
            # Boolean 

    def setup_folders(self):
        # Creating result, figure, folders if not in existence at specified paths
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)
        if not os.path.exists(self.figure_path):
            os.makedirs(self.figure_path)

    def setup_plotstyle(self):
        # Setting Plot Defaults
        plt.style.use('../../basic_plotter.mplstyle')

    def load_data(self, rate, interpolateM, processor):
        data_path = '../../data/'
        self.df = main.data_load(data_path, self.data_folder, self.file_name, rate, interpolateM, processor)
        return
    
    # Functions for defining the aircraft and the propulsion unit
    def define_propeller(self, prop_name):
        
        if prop_name == 'aeronaut20x8':
            self.propeller = propellers.aeronaut20x8()
        elif prop_name == 'aeronaut185x12':
            self.propeller = propellers.aeronaut185x12()
        elif prop_name == 'aeronaut11x7':
            self.propeller = propellers.aeronaut11x7_estimatedBEN
        else:
            raise ValueError("Need to add propeller to map or check name!")
        
    def define_motor(self, motor_name):
        
        if motor_name == 'U7V2_280KV':
            self.motor = motors.U7V2_280KV()
        elif motor_name == 'VOLANTEX_1050KV':
            self.motor = motors.VOLANTEX_1050KV()
        else:
            raise ValueError("Need to add propeller to map or check name!")

    def define_airframe(self, mass, chord, span):
        self.airframe = airplane(mass, chord, span)

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