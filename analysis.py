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

import decimal
import math

import plotly.express as px

import main
import propellers
import motors
from aircraft import airplane
import cl_finders

import data_exporter
import datetime

from matplotlib.widgets import SpanSelector

import pickle

data_path = '../../data/'

class Analysis:
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

        # Setting up folders and figure styles
        self.setup_folders()            # Creating folders if not already there
        self.setup_plotstyle()          # Setting plot defaults

        # Variables/Objects that will be used during analysis
        self.df = None                  # Pandas dataframe with Ardupilot logged data, through superwake recording
        self.data_map = None            # Dictionary for mapping variables from dataframe into the analysis 
        self.processor = None           # The name of the processor being used in the scope of the analysis
        self.method = Method()       # Object storing all interrim variables as a part of the analysis (Numpy arrays)
        self.result_data = None         # Object storing all pertinent results from the analysis

        self.airframe = None            # Airframe information, including mass, wing geometry (Class in aircraft.py under airplane)
        self.motor = None               # Motor information, includes theoretical model (Class in motors.py)
        self.propeller = None           # Propeller information, including thrust and efficiency models (Class in propellers.py)

    def select_segments(self, segment_times=None, year=None, month=None, day=None):
        # Generates an array of segment start/end times, and a list of boolean masks for each segment.  

        # either input an numpy array for interval selection, then generate the masks to store internally. Or start the graphical selection tool to start selecting parts.  
        # Check the results folder for a mask already there, use this if it is present. 

        if segment_times is None:
            # Starting selection process using GUI
            self.segment_times = np.empty((0,2), str)
            self.masks = np.array([])
            segment_counter = 0         # To increment as more segments are added

            # Creating index series to select data correctly
            indices = np.arange(0, self.df.shape[0])
            self.df["index_ref"] = indices

            # Beginning selection using plot of Airspeed and Altitude
            
            fig = plt.figure(figsize=(20,10), dpi=75)
            ax1 = plt.subplot(2,1,1)
            ax1.plot(self.df.index, self.df.Airspeed_Sensor0)
            ax2 = ax1.twiny()
            ax2.plot(self.df.index_ref, self.df.Airspeed_Sensor0, label='Airspeed 0')
            # ax2.plot(self.df.index_ref, self.df.Airspeed_Sensor1, label='Airspeed 1')
            # ax2.plot(self.df.index_ref, self.df.Groundspeed_GPS, label='Ground speed')
            ax2.plot(self.df.index_ref, self.df.Airspeed_Demanded, linestyle='solid', marker='', label='Demanded Airspeed')
            ax2.legend(loc="upper left")
            ax3 = plt.subplot(2, 1, 2, sharex=ax2)
            ax3.plot(self.df.index_ref, self.df.Altitude_GPS, label='GPS Altitude')
            ax3.legend(loc="upper left")

            def onselect(xmin, xmax):

                xmin = round(math.ceil(decimal.Decimal(xmin)))
                xmax = round(math.ceil(decimal.Decimal(xmax)))
                t_min = self.df.index[xmin]
                t_max = self.df.index[xmax]

                self.segment_interval = np.array([[str(t_min.hour).zfill(2)+':'+str(t_min.minute).zfill(2)+':'+str(t_max.second).zfill(2), str(t_max.hour).zfill(2)+':'+str(t_max.minute).zfill(2)+':'+str(t_max.second).zfill(2)]])
                self.segment_times = np.append(self.segment_times, self.segment_interval, axis=0)

                self.year = t_min.year
                self.month = t_min.month
                self.day = t_min.day

                self.masks = []                             # List of boolean masks that are stored for each segment (Need to generate)
                for i in range(np.shape(self.segment_times)[0]):
                    mask = cl_finders.get_mask(self.df, self.segment_times[i,0], self.segment_times[i,1], self.year, self.month, self.day)
                    self.masks.append(mask)

            self.span = SpanSelector(
                ax2,
                onselect,
                "horizontal",
                useblit=True,
                props=dict(alpha=0.5, facecolor="tab:blue"),
                interactive=True,
                drag_from_anywhere=True
            )

            plt.show()

        else:
            # Using the input segment times to generate a mask
            self.segment_times = segment_times          # Saving the segment time array for storage alongside the generated masks
            self.masks = []                             # List of boolean masks that are stored for each segment (Need to generate)
            self.year = year
            self.month = month
            self.day = day

            for i in range(np.shape(segment_times)[0]):
                mask = cl_finders.get_mask(self.df, segment_times[i,0], segment_times[i,1], year, month, day)
                self.masks.append(mask)
    
    # Function to save the selected segments to a pickle file in results
    def save_segments(self, segment_times, year, month, day, file_name, overwrite=False):
        segresult_name = self.result_path + file_name
        segment_package = segment_packaging(segment_times, year, month, day)

        if os.path.exists(segresult_name) and not overwrite:
            print("Segment file already exists, change to overwrite if necessary")
        else:
            with open(segresult_name, 'wb') as f:  # open a text file
                pickle.dump(segment_package, f)

    # Function to load the segments from the investigation already done
    def load_segments(self, file_name):
        segresult_name = self.result_path + file_name
        
        # Checking if the data is already pickled for analysis
        if os.path.exists(segresult_name):
            with open(segresult_name, 'rb') as f:
                segment_package = pickle.load(f) # deserialize using load()
            self.segment_times = segment_package.segment_times
            self.year = segment_package.year
            self.month = segment_package.month
            self.day = segment_package.day

        else:
            print("Error in finding segment file: " + segresult_name)

    # Creating folders for results and figures if not already existing
    def setup_folders(self):
        # Creating result, figure, folders if not in existence at specified paths
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)
        if not os.path.exists(self.figure_path):
            os.makedirs(self.figure_path)

    # Using a standard style of plot for matplotlib
    def setup_plotstyle(self):
        # Setting Plot Defaults
        plt.style.use('../../basic_plotter.mplstyle')

    # Loading data with specific rate, interpolation, and processor
    def load_data(self, rate, interpolateM, processor):
        self.processor = processor
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

    def find_datamap(self, processor):
        if processor == "CREATeV_2022":
            self.data_map = {
                # Position Variables
                "Latitude" :"Latitude",
                "Longitude":"Longitude",

                "Altitude":"Altitude",
                # Attitude Variables
                "Phi"      :"RollAngle", 
                "Theta"    :"PitchAngle",
                "Psi"      :"Heading",
                # Acceleration Variables 
                "Static_Pressure":"Pressure_BARO0",
                
                # Rotation Rate Variables
                "":"",

                # Velocity Variables

                # Drivetrain Variables
                "MotorRPM":"MotorRPM"
                }
    
    def add_method(self, analysis_type):
        analysis_list = ['cruise_eta', 'cruise_ct', 'gliding', 'acceleration']
        if analysis_type == "cruise_eta":
            self.method = Method.add_cruise_eta(self.df, self.data_map)
            self.result_data = self.method_data
        elif analysis_type == "cruise_ct":
            self.method_data = Method.add_cruise_ct(self.df, self.data_map)
        elif analysis_type == "gliding":
            self.method_data = Method.add_descent(self.df, self.data_map)
        elif analysis_type == "acceleration":
            self.method_data = Method.add_acceleration(self.df, self.data_map)
        else:
            print("Make sure the analysis type is one of the following: ")
            print(analysis_list)

class Method():
    def __init__(self):
        self.results = None              # Where the results of the method are stored
        self.data = None                # Where the raw variables are to be stored for the particular method
        return
    
    def add_cruise_eta(self, df, data_map):
        # Attitude 
        self.data.phi = np.deg2rad(df[data_map["Phi"]].to_numpy())            # Bank angle in radians
        self.data.pitch = np.deg2rad(df[data_map["Theta"]].to_numpy())         # Pitch angle in radians

        # Inertial Measurement Unit
        self.data.U_dot = df["XAcc_IMU"].to_numpy()                       # Acceleration in X direction

        # Atmospheric adjustments:
        self.data.rho = df["Pressure"].to_numpy() * (287 * (df["Ambient_Temperature"].to_numpy()+273.15))**-1             # Density found from barometer pressure & airspeed sensor temperatures
        self.data.v_eas = df["Airspeed_Sensor0"].to_numpy()                               # Equivalent SSL airspeed (m/s)
        self.data.v_tas = self.v_eas * np.sqrt(1.225) * np.sqrt(self.rho)**-1   # the true airspeed
        self.data.q = 0.5 * self.rho * self.v_tas**2                            # Dynamic pressure 

        # For Descent method
        self.data.h = df["Altitude_POS"].to_numpy()                   # Altitude
        self.data.Vd_eas = df["DescendingXK"].to_numpy()                  # Descent Rate from EKF (is it true or EAS at SSL?)
        self.data.Vd_tas = self.Vd_eas * np.sqrt(1.225) * np.sqrt(self.rho)**-1   # the true airspeed

        # Ground speed limiter
        self.data.v_dem = df["Airspeed_Demanded"].to_numpy()

        # Propulsion characterization
        self.data.n = df["MotorRPM"].to_numpy() / 60                               # Revolutions per second
        self.data.i_esc = df["EscCurrent"].to_numpy()
        self.data.v_esc = df["EscVoltage"].to_numpy()
        self.data.J = self.v_tas / (self.n * self.prop.diameter)
        self.data.eff = self.prop.efficiency(self.J) * self.motor.efficiency(self.n, self.i_esc)

        # Estimated propulsive power (New Fitting)
        self.data.P_eta = cl_finders.eta_steady(self.prop, self.motor, self.v_tas, self.n, self.i_esc, self.v_esc)
        self.data.P_ct = cl_finders.thrust_steady(self.prop, self.rho, self.v_tas, self.n)

        # Getting drag coefficient
        self.data.Cd_eta = cl_finders.preq2cd(self.aircraft, self.v_tas, self.q, self.P_eta)
        self.data.Cd_ct = cl_finders.preq2cd(self.aircraft, self.v_tas, self.q, self.P_ct)

        # Getting lift coefficient
        self.data.CL = cl_finders.cl_banked(self.aircraft, self.q, self.phi)
        return
    
    def add_descent(self, df, data_map):

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

    def add_acceleration(self, df, data_map):
        return
    
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
    
    def collect_bins(self, bins, total_lift_coeffs, total_drag_coeffs):
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
        cl_ci95s = [ 1.96 * (np.sqrt(len(digitized == i)))**-1 * total_lift_coeffs[digitized == i].std() for i in range(1, len(bins)) ]

        cd_means = [total_drag_coeffs[digitized == i].mean() for i in range(1, len(bins))]
        cd_stds = [total_drag_coeffs[digitized == i].std() for i in range(1, len(bins))]
        cd_ci95s = [ 1.96 * (np.sqrt(len(digitized == i)))**-1 * total_drag_coeffs[digitized == i].std() for i in range(1, len(bins)) ]

        cl_means = self.remove_nan(cl_means)
        cl_stds = self.remove_nan(cl_stds)
        cl_ci95s = self.remove_nan(cl_ci95s)
        cd_means = self.remove_nan(cd_means)
        cd_stds = self.remove_nan(cd_stds)
        cd_ci95s = self.remove_nan(cd_ci95s)

        return [cl_means, cl_stds, cl_ci95s, cd_means, cd_stds, cd_ci95s]
    
    def remove_nan(variable):
        variable = np.array(variable)
        variable = variable[~np.isnan(variable)]
        return variable

    # Function for packing the results, using the class defined above
    def packaging_results(cl_total, cl_means, cl_stds, cl_ci95s, cd_total, cd_means, cd_stds, cd_ci95s, polarfit, aircraft):
        # Packaging raw polars
        rawpolar = pd.DataFrame.from_dict({'CD': cd_total, 'CL': cl_total})
        # Packaging averaged polars
        avepolar = pd.DataFrame.from_dict({'CD': cd_means, 'CL': cl_means})
        # Packaging standard deviation polars
        stdpolar = pd.DataFrame.from_dict({'CD': cd_stds, 'CL': cl_stds})
        # Packaging 95% CI polars
        ci95polar = pd.DataFrame.from_dict({'CD': cd_ci95s, 'CL': cl_ci95s})

        package = result(rawpolar, avepolar, stdpolar, ci95polar, polarfit, aircraft)
        return package
    
class segment_packaging():
    def __init__(self, segment_times, year, month, day):
        self.segment_times = segment_times
        self.year = year
        self.month = month
        self.day = day

# Class that contains the final result of the analysis, often used
class result:
    def __init__(self, raw_polar, ave_polar, std_polar, ci95_polar, fit_polar, vehicle):
        self.raw_polar = raw_polar      # DataFrame with CL, CD data points
        self.ave_polar = ave_polar      # DataFrame with CL, CD averaged
        self.std_polar = std_polar      # DataFrame with Standard deviations of CL, CD
        self.ci95_polar = ci95_polar    # DataFrame with 95% confidence intervals calculated of CL, CD
        self.fit_polar = fit_polar      # DataFrame with two polar fits, one low order, one high order
        self.vehicle = vehicle          # Vehicle the data was taken with, should contain a) Mass, b) Wing area, c) Aspect Ratio