from External.AtmosphereData.DataFactory import dataFactory as df

baseFileName = "sw_index_2016_3hr.txt"

baseParams = df.fileParams("3 hour","Year Day Hour Ap Kp F107 ind1 ind2 ind3 ind4 ind5 ind6 ind7")
desParams = df.fileParams("1 day", "Year Day Hour Ap Kp F107 ind1 ind2 ind3 ind4 ind5 ind6 ind7")

options = df.options(False)

testFac = df.dataFactory(baseFileName, baseParams, desParams, options)

testFac.getBaseData()
print testFac.baseDict

testFac.baseToDes()
print testFac.desDict