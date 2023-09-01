#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
import csv
from Basilisk.architecture import sim_model

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
           while int(gravRow[0]) > currDeg:
               if( len(clmRow) < currDeg+1):
                   clmRow.extend([0.0]*(currDeg+1 - len(clmRow)))
                   slmRow.extend([0.0]*(currDeg+1 - len(slmRow)))
               clmList.append(clmRow)
               slmList.append(slmRow)
               clmRow = []
               slmRow = []
               currDeg += 1
           clmRow.append(float(gravRow[2]))
           slmRow.append(float(gravRow[3]))

        spherHarm.cBar = sim_model.MultiArray(clmList)
        spherHarm.sBar = sim_model.MultiArray(slmList)
        spherHarm.maxDeg = maxDeg


def loadGravFromFileToList(fileName, maxDeg=2):
    with open(fileName, 'r') as csvfile:
        gravReader = csv.reader(csvfile, delimiter=',')
        firstRow = next(gravReader)
        clmList = []
        slmList = []
        try:
            valCurr = int(firstRow[0])
        except ValueError:
            mu = float(firstRow[1])
            radEquator = float(firstRow[0])

        clmRow = []
        slmRow = []
        currDeg = 0
        for gravRow in gravReader:
            while int(gravRow[0]) > currDeg:
                if (len(clmRow) < currDeg + 1):
                    clmRow.extend([0.0] * (currDeg + 1 - len(clmRow)))
                    slmRow.extend([0.0] * (currDeg + 1 - len(slmRow)))
                clmList.append(clmRow)
                slmList.append(slmRow)
                clmRow = []
                slmRow = []
                currDeg += 1
            clmRow.append(float(gravRow[2]))
            slmRow.append(float(gravRow[3]))

        return [clmList, slmList, mu, radEquator]
        
        
def loadPolyFromFile(fileName, poly):
    with open(fileName) as polyFile:
        if fileName.endswith('.tab'):
            try:
                nVertex, nFacet = [int(x) for x in next(polyFile).split()] # read first line
                fileType = 'gaskell'
            except:
                polyFile.seek(0)
                fileType = 'pds3'

            vertList = []
            faceList = []
            if fileType == 'gaskell':
                contLines = 0
                for line in polyFile:
                    arrtemp = []

                    for x in line.split():
                        arrtemp.append(float(x))

                    if contLines < nVertex:
                        vertList.append([float(arrtemp[1]*1e3),float(arrtemp[2]*1e3),float(arrtemp[3]*1e3)])
                    else:
                        faceList.append([int(arrtemp[1]),int(arrtemp[2]),int(arrtemp[3])])

                    contLines += 1
            elif fileType == 'pds3':
                nVertex = 0
                nFacet = 0
                vertList = []
                faceList = []
                for line in polyFile:
                    arrtemp = line.split()
                    if arrtemp:
                        if arrtemp[0] == 'v':
                            nVertex += 1
                            vertList.append([float(arrtemp[1])*1e3, float(arrtemp[2])*1e3, float(arrtemp[3])*1e3])
                        elif arrtemp[0] == 'f':
                            nFacet += 1
                            faceList.append([int(arrtemp[1])+1, int(arrtemp[2])+1, int(arrtemp[3])+1])
        elif fileName.endswith('.obj'):
            nVertex = 0
            nFacet = 0
            vertList = []
            faceList = []
            for line in polyFile:
                arrtemp = line.split()
                if arrtemp:
                    if arrtemp[0] == 'v':
                        nVertex += 1
                        vertList.append([float(arrtemp[1])*1e3, float(arrtemp[2])*1e3, float(arrtemp[3])*1e3])
                    elif arrtemp[0] == 'f':
                        nFacet += 1
                        faceList.append([int(arrtemp[1]), int(arrtemp[2]), int(arrtemp[3])])
        elif fileName.endswith('.txt'):
            nVertex, nFacet = [int(x) for x in next(polyFile).split()] # read first line
            vertList = []
            faceList = []

            contLines = 0
            for line in polyFile:
                arrtemp = []

                for x in line.split():
                    arrtemp.append(float(x))

                if contLines < nVertex:
                    vertList.append([float(arrtemp[0]*1e3),float(arrtemp[1]*1e3),float(arrtemp[2]*1e3)])
                else:
                    faceList.append([int(arrtemp[0]),int(arrtemp[1]),int(arrtemp[2])])

                contLines += 1

        poly.nVertex = nVertex
        poly.nFacet = nFacet
        poly.xyzVertex = vertList
        poly.orderFacet = faceList


def loadPolyFromFileToList(fileName):
    with open(fileName) as polyFile:
        if fileName.endswith('.tab'):
            try:
                nVertex, nFacet = [int(x) for x in next(polyFile).split()] # read first line
                fileType = 'gaskell'
            except:
                polyFile.seek(0)
                fileType = 'pds3'

            if fileType == 'gaskell':
                vertList = []
                faceList = []

                contLines = 0
                for line in polyFile:
                    arrtemp = []

                    for x in line.split():
                        arrtemp.append(float(x))

                    if contLines < nVertex:
                        vertList.append([float(arrtemp[1]*1e3),float(arrtemp[2]*1e3),float(arrtemp[3]*1e3)])
                    else:
                        faceList.append([int(arrtemp[1]),int(arrtemp[2]),int(arrtemp[3])])

                    contLines += 1
            elif fileType == 'pds3':
                nVertex = 0
                nFacet = 0
                vertList = []
                faceList = []
                for line in polyFile:
                    arrtemp = line.split()
                    if arrtemp:
                        if arrtemp[0] == 'v':
                            nVertex += 1
                            vertList.append([float(arrtemp[1])*1e3, float(arrtemp[2])*1e3, float(arrtemp[3])*1e3])
                        elif arrtemp[0] == 'f':
                            nFacet += 1
                            faceList.append([int(arrtemp[1])+1, int(arrtemp[2])+1, int(arrtemp[3])+1])
        elif fileName.endswith('.obj'):
            nVertex = 0
            nFacet = 0
            vertList = []
            faceList = []
            for line in polyFile:
                arrtemp = line.split()
                if arrtemp:
                    if arrtemp[0] == 'v':
                        nVertex += 1
                        vertList.append([float(arrtemp[1])*1e3, float(arrtemp[2])*1e3, float(arrtemp[3])*1e3])
                    elif arrtemp[0] == 'f':
                        nFacet += 1
                        faceList.append([int(arrtemp[1]), int(arrtemp[2]), int(arrtemp[3])])
        elif fileName.endswith('.txt'):
            nVertex, nFacet = [int(x) for x in next(polyFile).split()] # read first line
            vertList = []
            faceList = []

            contLines = 0
            for line in polyFile:
                arrtemp = []

                for x in line.split():
                    arrtemp.append(float(x))

                if contLines < nVertex:
                    vertList.append([float(arrtemp[0]*1e3),float(arrtemp[1]*1e3),float(arrtemp[2]*1e3)])
                else:
                    faceList.append([int(arrtemp[0]),int(arrtemp[1]),int(arrtemp[2])])

                contLines += 1

        return [vertList, faceList, nVertex, nFacet]
