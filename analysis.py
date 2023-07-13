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

    def setup_folders(self):
        # Creating result, figure, folders if not in existence at specified paths
        if not os.path.exists(self.result_path):
            os.makedirs(self.result_path)
        if not os.path.exists(self.figure_path):
            os.makedirs(self.figure_path)

    def setup_plotstyle(self):
        # Setting Plot Defaults
        plt.style.use('../basic_plotter.mplstyle')

    def load_data(self, rate, interpolateM, processor):
        data_path = '../data/'
        self.df = main.data_load(data_path, self.data_folder, self.file_name, rate, interpolateM, processor)
        return
    
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

    def define_airframe(self, airframe):
        self.vehicle = airframe