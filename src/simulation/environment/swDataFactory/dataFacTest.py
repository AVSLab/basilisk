import swDataFactory as df
from matplotlib import pyplot as plt
import numpy as np

baseFileName = "sw_index_2016_3hr.txt"

baseParams = df.fileParams("3 hour","Year Day Hour Ap Kp F107 ind1 ind2 ind3 ind4 ind5 ind6 ind7", " ")
desParams = df.fileParams("12 hour", "Year Day Hour Ap Kp F107 ind1 ind2 ind3 ind4 ind5 ind6 ind7", " ")

options = df.options(False)

testFac = df.swDataFactory()

testFac.getBaseData()
#print testFac.baseDict

testFac.baseToDes()
#print testFac.desDict['Ap']

plt.figure()
plt.plot(testFac.desDict['Ap'])
plt.title('Desired Ap')

plt.figure()
plt.plot(testFac.baseDict['Ap'])
plt.title('Base Values')

timeVec = np.arange(0,48*3600.0,60)
doy = 1
testInterp = []

for ind in timeVec:
    testInterp.append(testFac.getVarVal( doy,int(ind/3600),int(ind/60), ind - int(ind/3600)*3600.0 - int(ind/60)*60, "Ap"))

plt.figure()
plt.plot(testInterp)
plt.title('Interpolation test')
plt.show()