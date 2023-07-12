import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import pickle
from superwake_recording import RecordingLoader

def data_load(data_path, data_folder, file_name, rate, interpolateM, processor):

    rawdata_name = data_folder+'/'+file_name.split('.')[0]+'_'+rate+'_'+interpolateM+'_'+processor+'.pkl'

    # Checking if the data is already pickled for analysis
    if os.path.exists(data_path+rawdata_name):
        df = pd.read_pickle(data_path+rawdata_name)
    else:
        df = get_data(processor, data_folder+'/'+file_name, rate)
        df = df.interpolate(method=interpolateM)
        pd.to_pickle(df, data_path+rawdata_name) # Storing parsed data
    return df

def save_figure(figure, plot_name, path, overwrite=False):
    # Saves figure to pdf and into pkl for opening later
    if overwrite:
        plt.savefig(path+plot_name+'.pdf', bbox_inches='tight')
        pickle.dump(figure, open(path+plot_name+'.pkl', 'wb'))
    elif os.path.exists(path+plot_name+'.pdf'):
        print("Plot already exists")
    else:
        plt.savefig(path+plot_name+'.pdf', bbox_inches='tight')
        pickle.dump(figure, open(path+plot_name+'.pkl', 'wb'))

def open_figure(plot_name, path):
    # Opens figure compressed to pkl file
    figure = pickle.load(open(path+plot_name+'.pkl','rb'))
    figure.suptitle(plot_name)
    return figure

def get_data(method, file_name, rate):
    
    #  Stating absolute path to data folder 
    root = "../data/"
    #  Creating path to individual file
    file_path = root + file_name

    if method == "Alton":
        #  Selecting processor based on filename
        if "2022" in file_name:
            recording = RecordingLoader.load("CREATeV_2022", file_path)
        elif "2023" in file_name:
            recording = RecordingLoader.load("ArduPlane_4_3_3", file_path)
        elif "2021" in file_name:
            recording = RecordingLoader.load("CREATeV_2021", file_path)
        else:
            print("Error in data name, make sure it has the year somewhere (eg 2021, 2022")
    else:
        recording = RecordingLoader.load(method, file_path)
    # elif method == "CREATeV_2023":
    #     recording = RecordingLoader.load(method, file_path)
    # elif method == "CREATeV_2022":
    #     recording = RecordingLoader.load(method, file_path)
    # elif method == "ArduPlane_4_3_3":
    #     recording = RecordingLoader.load(method, file_path)
    # elif method == "CREATeV_2021":
    #     recording = RecordingLoader.load(method, file_path)
    # else:
    #     print("Error importing data")
    #    return
    #  Resampling data to desired rate
    df = recording.resample(rate)
    return df

class flight:
    def __init__(self, propeller, motor, airplane, dataframe):
        
        #  Flight properties
        self.propeller = propeller  # Containing propeller performance models 
        self.motor = motor          # Containing motor performance models
        self.airplane = airplane    # Containing variables intrinsic to CREATeV
        self.data = dataframe       # Containing imported BIN file

class result:
    def __init__(self, raw_polar, ave_polar, std_polar, ci95_polar, fit_polar, vehicle):
        self.raw_polar = raw_polar      # DataFrame with CL, CD data points
        self.ave_polar = ave_polar      # DataFrame with CL, CD averaged
        self.std_polar = std_polar      # DataFrame with Standard deviations of CL, CD
        self.ci95_polar = ci95_polar    # DataFrame with 95% confidence intervals calculated of CL, CD
        self.fit_polar = fit_polar      # DataFrame with two polar fits, one low order, one high order
        self.vehicle = vehicle          # Vehicle the data was taken with, should contain a) Mass, b) Wing area, c) Aspect Ratio

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

    package = result(rawpolar, avepolar, stdpolar, ci95polar, polarfit, aircraft)
    return package

class analysis:
    def __init__(self, flight, test_name):
        self.type = test_name
        self.flight = flight                                # Importing flight information
        self.init_masks()                                   # Initializing static masks

        self.mask = self.select_mask(flight, test_name)     # Selecting mask based on 

    def init_masks(self):
        #  Initialized manually input masks for data selection
        june15_loiter = np.array([[9000, 9600],
                                  [9600, 10200],
                                  [10200,10900],
                                  [10900,11500],
                                  [11500,11900],
                                  [12800,13350],
                                  [13400,13950],
                                  [14050,14450]])

        june15_descending = np.array([[3440, 3500],
                                      [3680, 3746],
                                      [3920, 3993],
                                      [4158, 4237],
                                      [4412, 4488],
                                      [4650, 4740],
                                      [4910, 4992],
                                      [5154, 5240],
                                      [5391, 5478],
                                      [5646, 5746],
                                      [5920, 6008],
                                      [6193, 6284],
                                      [6462, 6556],
                                      [6742, 6850],
                                      [7037, 7156],
                                      [7344, 7462],
                                      [7672, 7774],
                                      [7980, 8082],
                                      [8300, 8428],
                                      [8576, 8694]])
                   
        june13_loiter = np.array([[5750,6340],
                                  [6350,6940],
                                  [6950,7540],
                                  [7550,8140],
                                  [8150,8740],
                                  [8750,9340],
                                  [9350,9940],
                                  [9950,10540]])

        self.masks = {
            "loiter_june15_2022.BIN": june15_loiter,
            "descending_june15_2022.BIN": june15_descending,
            "loiter_june13_2022.BIN": june13_loiter
        }
