#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  4 14:34:44 2021

@author: williamkemp
"""

import pandas as pd
from scipy import io
import numpy as np
import h5py

def import_gndstation(name):
    
    # Input is the name of the groundstation datafile, usually LOGGER05
    
    # Atmospheric Data Import

    atm_data = pd.read_csv(name +'.CSV', sep=',')
    atm_data = atm_data.fillna(0.0)
    atm_data = atm_data.replace(-9999.0, 0.0)
    atm_data = atm_data.to_numpy()
    
    pressure = atm_data[:, 5] # In Pa
    tempf = atm_data[:, 4] # In Farenheit
    windspeed = atm_data[:, 2] # In m/s, some outliers had to be removed (-9999.0)
    
    # Need to replace every nth element with average, since 0's prevade the dataspace
    
    pressure[14::15] = pressure[13::15]
    tempf[14::15] = tempf[13::15]
    tempc = (tempf - 32) * (5/9)
    
    gnd_data = {'pressure': pressure, 'tempc': tempc, 'windspeed': windspeed}
    return gnd_data

def import_flt(name):
    
    mat = io.loadmat(name +'.mat', struct_as_record=False, squeeze_me=True)
    fmt = mat.get('FMT')    
    
    return fmt

def import_flthdf(name):
    
    with h5py.File(name+'.mat', 'r') as f:
        f.keys()