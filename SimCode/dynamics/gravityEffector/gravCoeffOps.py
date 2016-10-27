import csv
import sim_model

def loadGravFromFile(fileName, spherHarm, maxDeg=2):

    with open(fileName, 'r') as csvfile:
        gravReader = csv.reader(csvfile, delimiter=',')
        firstRow = next(gravReader)
        clmList = []
        slmList = []
        try:
            valCurr = int(firstRow[0])
        except ValueError:
            spherHarm.muBody = float(firstRow[1])
            spherHarm.radEquator = float(firstRow[0])

        clmRow = []
        slmRow = []
        currDeg = 0
        for gravRow in gravReader:
           if int(gravRow[0]) != currDeg:
               clmList.append(clmRow)
               slmList.append(slmRow)
               clmRow = []
               slmRow = []
               currDeg = int(gravRow[0])
           clmRow.append(float(gravRow[2]))
           slmRow.append(float(gravRow[3]))
    
        spherHarm.cBar = sim_model.MultiArray(clmList)
        spherHarm.sBar = sim_model.MultiArray(slmList)
        spherHarm.maxDeg = maxDeg

