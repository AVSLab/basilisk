#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 24 19:59:42 2021

@author: mikaelafelix
"""

import numpy as np
import pandas as pd


def readAtmTable(filename,typename):
    if typename.lower() == 'earthgram':
        altList, rhoList, tempList = readEarthGRAM(filename)
    elif typename.lower() == 'stdatm76':
        altList, rhoList, tempList = readStdAtm76(filename)
    elif typename.lower() == 'msis':
        altList, rhoList, tempList = readMSIS(filename)
    elif typename.lower() == 'marsgram':
        altList, rhoList, tempList = readMarsGRAM(filename)
    elif typename.lower() == 'venusgram':
        altList, rhoList, tempList = readVenusGRAM(filename)
    elif typename.lower() == 'uranusgram':
        altList, rhoList, tempList = readUranusGRAM(filename)
    elif typename.lower() == 'titangram':
        altList, rhoList, tempList = readTitanGRAM(filename)
    elif typename.lower() == 'jupitergram':
        altList, rhoList, tempList = readJupiterGRAM(filename)
    else:
        print('Type not recognized')
        return np.nan, np.nan, np.nan

    return altList, rhoList, tempList

def readStdAtm76(filename):
    df = pd.read_csv(filename, skiprows=[1])
    df.sort_values(by=['Altitude'], ascending = True, inplace=True)
    altList = df.Altitude.to_list()
    tempList = df.Temperature.to_list()
    rhoList = df.Density.to_list()
    return altList, rhoList, tempList


def readEarthGRAM(filename):
    df = pd.read_csv(filename, sep=r'\s+')
    df.sort_values(by=['Hgtkm'],ascending=True, inplace=True)
    df.Hgtkm = df.Hgtkm * 1000
    altList = df.Hgtkm.to_list()
    rhoList = df.DensMean.to_list()
    tempList = df.Tmean.to_list()

    return altList, rhoList, tempList

def readMarsGRAM(filename):
	df = pd.read_csv(filename, sep=r'\s+')
	df.sort_values(by=['HgtMOLA'],ascending=True, inplace=True)
	df.HgtMOLA = df.HgtMOLA * 1000
	altList = df.HgtMOLA.to_list()
	rhoList = df.Denkgm3.to_list()
	tempList = df.Temp.to_list()

	return altList, rhoList, tempList


def readVenusGRAM(filename):
	df = pd.read_csv(filename, skiprows=[1])
	df.sort_values(by=['Height_km'],ascending=True, inplace=True)
	df.Height_km = df.Height_km * 1000
	altList = df.Height_km.to_list()
	rhoList = df.Density_kgm3.to_list()
	tempList = df.Temperature_K.to_list()

	return altList, rhoList, tempList


def readUranusGRAM(filename):
	df = pd.read_csv(filename, skiprows=[1])
	df.sort_values(by=['Height_km'],ascending=True, inplace=True)
	df.Height_km = df.Height_km * 1000
	altList = df.Height_km.to_list()
	rhoList = df.Density_kgm3.to_list()
	tempList = df.Temperature_K.to_list()

	return altList, rhoList, tempList


def readTitanGRAM(filename):
	df = pd.read_csv(filename, skiprows=[1])
	df.sort_values(by=['Height_km'],ascending=True, inplace=True)
	df.Height_km = df.Height_km * 1000
	altList = df.Height_km.to_list()
	rhoList = df.Density_kgm3.to_list()
	tempList = df.Temperature_K.to_list()

	return altList, rhoList, tempList


def readJupiterGRAM(filename):
	df = pd.read_csv(filename, skiprows=[1])
	df.sort_values(by=['Height_km'],ascending=True, inplace=True)
	df.Height_km = df.Height_km * 1000
	altList = df.Height_km.to_list()
	rhoList = df.Density_kgm3.to_list()
	tempList = df.Temperature_K.to_list()

	return altList, rhoList, tempList


def readMSIS(filename):
    df = pd.read_csv(filename, skiprows = 29, header=0, sep=r'\s+',
                   names=["alt", "rho", "temp"])
    df.sort_values(by=['alt'],ascending=True, inplace=True)
    df.alt = df.alt * 1000
    altList = df.alt.to_list()
    df.rho = df.rho / 1000 * 100**3
    rhoList = df.rho.to_list()
    tempList = df.temp.to_list()

    return altList, rhoList, tempList
