import numpy as np
import matplotlib.pyplot as plt
from superwake_recording import RecordingLoader

class flight:

    def __init__(self, propeller_name, test_types, aircraft_mass, file):
        
        #  Flight properties
        self.propeller = propeller_name
        self.mass = aircraft_mass
        self.test_types = test_types  #  Tests conducted during flight, or to be analyzed

        #  Test object storage init
        self.tests = []

        #  Initialized/Loaded flight data
        self.file_name = file
        self.dataframe = self.get_data("Alton")

        #  Initializing test objects

        for i in range(self.test_types):
            self.tests.append(test_obj(self, test_types[i]))

    def get_data(self, method):

        if method == "Alton":
            #  Stating absolute path to data folder 
            root = "/Users/williamkemp/Records/Repositories/CREATeV_power/data/"
            #  Creating path to individual file
            file_path = root + self.file_name

            #  Selecting processor based on filename
            if "2022" in self.file_name:
                recording = RecordingLoader.load("CREATeV_2022", file_path)
            elif "2021" in self.file_name:
                recording = RecordingLoader.load("CREATeV_2021", file_path)
            else:
                print("Error in data name, make sure it has the year somewhere (eg 2021, 2022")
            
            #  Resampling data to desired rate
            df = recording.resample("1s")
            return df
        else:
            x = None
            #  Use my own data importer

    def visualize(self):
        
        #  Plot for Descending information
        plt.figure(1)
        ax1 = subplot(3,1,1)
        ax1.plot(self.dataframe["Airspeed"])
        ax1.set_xlabel("Time")
        ax1.set_ylabel("Airspeed (m/s)")

        ax2 = subplot(3,1,2, sharex=ax1)
        ax2.plot(self.dataframe["DescendingXK"])
        ax2.set_xlabel("Time")
        ax2.set_ylabel("Descent Rate (m/s)")

        ax3 = subplot(3,1,3, sharex=ax1)
        ax3.plot(self.dataframe["Altitude"])
        ax3.set_xlabel("Time")
        ax3.set_ylabel("Altitude (m)")

        #  Plot for Loiter information
        plt.show()

class test_obj:

    def __init__(self, flight, test_name):
        self.type = test_name
        self.flight = flight
        self.init_masks()

        self.mask = self.select_mask(flight, test_name)

    def init_masks(self):
        
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

    def select_mask(self, flight, test_name):
        
        key = test_name + '_' + flight
        return self.masks[key]

    def main_analysis(self, flight_dataframe):
        if self.type == 'loiter':
            if self.flight.propeller == '20x8':

            elif self.flight.propeller == '185x12':

class propeller:
    # Class containing the aerodynamic information of the propeller
    def __init__(self):
        self.name = 
class esc:
    # Class containing the efficiency information of the Electronic speed controller
    self.name

        elif self.type == 'descending':

        


june15_2022 = flight("185x12", ['loiter', 'descending'], 12.7, "june15_2022.BIN")
june13_2022 = flight("20x8", ['loiter'], 12.7, "june13_2022.BIN")
august04_2021 = flight("20x8", ['loiter'], 12.6, "august04_2021")
