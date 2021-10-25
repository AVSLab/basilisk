#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 24 19:59:42 2021

@author: mikaelafelix
"""

import csv
import pandas as pd
import numpy as np

def readAtmTable(filename,typename):
    if typename.lower() == 'earthgram':
        altList, rhoList, tempList = readEarthGRAM(filename)
    elif typename.lower() == 'stdatm76':
        altList, rhoList, tempList = readStdAtm76(filename)
    else:
        print('Type not recognized')
        return np.NaN, np.NaN, np.NaN
    
    return altList, rhoList, tempList

def readStdAtm76(filename):
    df = pd.read_csv(filename, skiprows=[1])
    df.sort_values(by=['Altitude'], ascending = True, inplace=True)
    altList = df.Altitude.to_list()
    tempList = df.Temperature.to_list()
    rhoList = df.Density.to_list()
    return altList, rhoList, tempList
    
    
def readEarthGRAM(filename):
    df = pd.read_csv(filename, delim_whitespace = True) 

    df.sort_values(by=['Hgtkm'],ascending=True, inplace=True)
    df.Hgtkm = df.Hgtkm * 1000
    altList = df.Hgtkm.to_list()
    rhoList = df.DensMean.to_list()
    tempList = df.Tmean.to_list()
    
    return altList, rhoList, tempList

    
myfilename = '../../supportData/AtmosphereData/USStandardAtmosphere1976.csv'

altList,rhoList,tempList = readAtmTable(myfilename, 'stdatm76')