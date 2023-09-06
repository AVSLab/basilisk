To include the WMM code, note that the following lines had to be changed to replace 

	*(*magneticmodels)[]

With 

	*(*magneticmodels)[1]

- MAG_robustReadMagModels()
- MAG_readMagneticModel_SHDF()
- MAG_PrintSHDFFormat()

========
Append 'f' float suffix to each value in 'float GeoidHeightBuffer' in EGM9615.h