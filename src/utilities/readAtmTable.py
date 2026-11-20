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


def _read_gram_csv(filename):
    units = {
        "Height_km": ("km", "[km]"),
        "Density_kgm3": ("kg/m3", "[kg/m3]", "kg/m^3", "[kg/m^3]", "kg/m³", "[kg/m³]"),
        "Temperature_K": ("K", "[K]", "Kelvin", "[Kelvin]"),
    }
    df = pd.read_csv(filename, usecols=list(units), keep_default_na=False)
    if not df.empty and all(str(df[column].iat[0]).strip() in labels for column, labels in units.items()):
        df = df.iloc[1:]
    df = df.astype(float).sort_values("Height_km")
    altitude = df["Height_km"] * 1000.0  # [m]
    return altitude.to_list(), df["Density_kgm3"].to_list(), df["Temperature_K"].to_list()


def readVenusGRAM(filename):
    return _read_gram_csv(filename)


def readUranusGRAM(filename):
    return _read_gram_csv(filename)


def readTitanGRAM(filename):
    return _read_gram_csv(filename)


def readJupiterGRAM(filename):
    return _read_gram_csv(filename)


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
