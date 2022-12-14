AtmosphereData documentation
Sam Albert
Last updated: Nov. 1 2021

===============================================================================
Overview
===============================================================================
The directory supportData/AtmosphereData provides a number of data
files to be used by the TabularAtmosphere module. This module interpolates
from a user-provided table of altitude, density, and temperature. 

Any user-supplied data that is in ascending order and units of meters, kg/m^3,
and Kelvin will work.

For convenience, a number of data tables generated by common atmosphere models
are included in this directory. Utility functions are then provided in python
in the file src/utilities/readAtmTable.py to read in each of these files.

The provided python functions do the conversions from whatever units are used
in these data files to m, kg/m^3, and K.

This file provides notes on how these pre-supplied data tables were generated.
Supporting files are also included in this directory as needed. All of the data
provided here are from publicly available sources and not export-controlled.


===============================================================================
USStandardAtmosphere1976.csv
===============================================================================
The 1976 version of the US Standard Atmosphere is defined in a report of the
same name from NOAA, NASA, and the USAF, currently available at the link below:
 https://www.ngdc.noaa.gov/stp/space-weather/online-publications/...
     miscellaneous/us-standard-atmosphere-1976/...
     us-standard-atmosphere_st76-1562_noaa.pdf 

The provided data table was generated using the webapp available at:
https://www.digitaldutch.com/atmoscalc/table.htm

Temperature and Density were output in Kelvin and kilograms/cubic meter, 
respectively. A minimum altitude of -5000 m and max of 86000 m were used with a
500 meter increment.

NOTE: the US Standard Atmosphere is not defined above an altitude of 86 km,
limiting its usefulness for some entry applications.


===============================================================================
EarthGRAMNominal.txt
===============================================================================
The 2016 version of the Global Reference Atmospheric Model for Earth, or
Earth-GRAM2016, was used to generate this data. GRAM software is under General
Public Release from NASA but must be requested for download: 
https://software.nasa.gov/software/MFS-32780-2

GRAM is run with numerous required inputs included in a namelist file. The file
used to generate the data included here is provided in this directory as 
EarthGRAM_bsk_namelist.txt, which also includes brief descriptions of each
input variable. See the GRAM documentation for more info.

In the provided file, Hgtkm is altitude in kilometers, DensMean is the mean
density in kilograms/cubic meter, and Tmean is the mean temperature in Kelvin.
Perturbed values are also included, but for this nominal output the mean values
are more appropriate, and these are what is read in by the python function.

The provided data is a single columnar (only altitude varies) atmosphere table.
To generate N dispersed perturbed profiles with the same settings, one could
change the value of mc in the namelist file to equal N and re-run.


===============================================================================
NRLMSISE00Nominal.txt
===============================================================================
The 2000 version of the US Naval Research Laboratory mass spectrometer and 
incoherent radar (exosphere), or NRLMSISE-00, was used to generate this data.
The sourcecode is available for download and Basilisk also has functionality to
run MSIS-in-the-loop for more precise density values. As an additional option,
this data file is a single nominal data table for a columnar atmosphere. The
data was generated using the webapp available at: 
https://ccmc.gsfc.nasa.gov/modelweb/models/nrlmsise00.php

The inputs, also listed in the header of the data file, are: date of 4/1/21 at
21 hour of the day, 40 deg geographic latitude, & 0-500 km altitude in steps of
0.5 km, with Height as the single independent variable.

In the provided data file, 1 is altitude in kilometers, 2 is density in
grams/cubic centimeter, and 3 is temperature in Kelvin.

===============================================================================
MarsGRAMNominal.txt
===============================================================================
The 2010 version of the Global Reference Atmospheric Model for Mars, or 
Mars-GRAM2010, was used to generate this data. GRAM software is under General
Public Release from NASA but must be requested for download: 
https://software.nasa.gov/software/MFS-33158-1

GRAM is run with numerous required inputs included in a namelist file. The file
used to generate the data included here is provided in this directory as 
MarsGRAM_bsk_namelist.txt, which also includes brief descriptions of each
input variable. See the GRAM documentation for more info.

In the provided file, HgtMOLA is altitude in kilometers, Denkgm3 is the mean
density in kilograms/cubic meter, and Temp is the avg. temperature in Kelvin.
Perturbation values are also included, but for this nominal output the mean 
values are more appropriate, & these are what's read in by the python function.

The provided data is a single columnar (only altitude varies) atmosphere table.
To generate N dispersed perturbed profiles with the same settings, one could
change the value of NMONTE in the namelist file to equal N and re-run.


===============================================================================
VenusGRAMNominal.csv
===============================================================================
The 2021 version of the Global Reference Atmospheric Model for Venus, or 
Venus-GRAM2021, was used to generate this data. GRAM software is under General
Public Release from NASA but must be requested for download: 
https://software.nasa.gov/software/MFS-33888-1

GRAM is run with numerous required inputs included in a namelist file. The file
used to generate the data included here is provided in this directory as 
VenusGRAM_bsk_namelist.txt, which also includes brief descriptions of each
input variable. See the GRAM documentation for more info.

In the provided file, Height_km is altitude above the reference ellipsoid in
kilometers, Density_kgm3 is the mean density in kilograms/cubic meter, and
Temperature_K is the avgerage temperature in Kelvin. Perturbation values are
also included, but for this nominal output the mean values are more
appropriate, and these are what is read in by the python function.

The provided data is a single columnar (only altitude varies) atmosphere table.
To generate N dispersed perturbed profiles with the same settings, one could
change the value of NumberOfMonteCarloRuns in the namelist file to equal N
and re-run.


===============================================================================
TitanGRAMNominal.csv
===============================================================================
The 2021b version of the Global Reference Atmospheric Model for Titan, or 
Titan-GRAM2021b, was used to generate this data. GRAM software is under General
Public Release from NASA but must be requested for download: 
https://software.nasa.gov/software/MFS-33888-1

GRAM is run with numerous required inputs included in a namelist file. The file
used to generate the data included here is provided in this directory as 
TitanGRAM_bsk_namelist.txt, which also includes brief descriptions of each
input variable. See the GRAM documentation for more info.

In the provided file, Height_km is altitude above the reference ellipsoid in
kilometers, Density_kgm3 is the mean density in kilograms/cubic meter, and
Temperature_K is the avgerage temperature in Kelvin. Perturbation values are
also included, but for this nominal output the mean values are more
appropriate, and these are what is read in by the python function.

The provided data is a single columnar (only altitude varies) atmosphere table.
To generate N dispersed perturbed profiles with the same settings, one could
change the value of NumberOfMonteCarloRuns in the namelist file to equal N
and re-run.

===============================================================================
NeptuneGRAMNominal.csv
===============================================================================
The 2019c version of the Global Reference Atmospheric Model for Neptune, or 
Neptune-GRAM2019c, was used to generate this data. GRAM software is under
General Public Release from NASA but must be requested for download: 
https://software.nasa.gov/software/MFS-33888-1

GRAM is run with numerous required inputs included in a namelist file. The file
used to generate the data included here is provided in this directory as 
NeptuneGRAM_bsk_namelist.txt, which also includes brief descriptions of each
input variable. See the GRAM documentation for more info.

In the provided file, Height_km is altitude above the reference ellipsoid in
kilometers, Density_kgm3 is the mean density in kilograms/cubic meter, and
Temperature_K is the avgerage temperature in Kelvin. Perturbation values are
also included, but for this nominal output the mean values are more
appropriate, and these are what is read in by the python function.

The provided data is a single columnar (only altitude varies) atmosphere table.
To generate N dispersed perturbed profiles with the same settings, one could
change the value of NumberOfMonteCarloRuns in the namelist file to equal N
and re-run.

===============================================================================
UranusGRAMNominal.csv
===============================================================================
The 2021a version of the Global Reference Atmospheric Model for Uranus, or 
Uranus-GRAM2021a, was used to generate this data. GRAM software is under
General Public Release from NASA but must be requested for download: 
https://software.nasa.gov/software/MFS-33888-1

GRAM is run with numerous required inputs included in a namelist file. The file
used to generate the data included here is provided in this directory as 
UranusGRAM_bsk_namelist.txt, which also includes brief descriptions of each
input variable. See the GRAM documentation for more info.

In the provided file, Height_km is altitude above the reference ellipsoid in
kilometers, Density_kgm3 is the mean density in kilograms/cubic meter, and
Temperature_K is the avgerage temperature in Kelvin. Perturbation values are
also included, but for this nominal output the mean values are more
appropriate, and these are what is read in by the python function.

The provided data is a single columnar (only altitude varies) atmosphere table.
To generate N dispersed perturbed profiles with the same settings, one could
change the value of NumberOfMonteCarloRuns in the namelist file to equal N
and re-run.

===============================================================================
JupiterGRAMNominal.csv
===============================================================================
The 2021 version of the Global Reference Atmospheric Model for Jupiter, or 
Jupiter-GRAM2021, was used to generate this data. GRAM software is under
General Public Release from NASA but must be requested for download: 
https://software.nasa.gov/software/MFS-33888-1

GRAM is run with numerous required inputs included in a namelist file. The file
used to generate the data included here is provided in this directory as 
JupiterGRAM_bsk_namelist.txt, which also includes brief descriptions of each
input variable. See the GRAM documentation for more info.

In the provided file, Height_km is altitude above the reference ellipsoid in
kilometers, Density_kgm3 is the mean density in kilograms/cubic meter, and
Temperature_K is the avgerage temperature in Kelvin. Perturbation values are
also included, but for this nominal output the mean values are more
appropriate, and these are what is read in by the python function.

JupiterGRAM does not currently include a perturbation model for use in Monte
Carlo analyses.