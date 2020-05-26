import math
import numpy as np
from Basilisk.fswAlgorithms import fswMessages


def create_rw_lists():
    RWAGsMatrix = []
    RWAJsList = []
    wheelJs = 50.0 / (6000.0 / 60.0 * math.pi * 2.0)
    rwElAngle = 42.5 * math.pi / 180.0

    rwClockAngle = 45.0 * math.pi / 180.0
    RWAJsList.extend([wheelJs])
    RWAGsMatrix.extend([
        math.sin(rwElAngle) * math.sin(rwClockAngle),
        math.sin(rwElAngle) * math.cos(rwClockAngle),
        -math.cos(rwElAngle)
    ])

    rwClockAngle += 90.0 * math.pi / 180.0
    RWAJsList.extend([wheelJs])
    RWAGsMatrix.extend([math.sin(rwElAngle) * math.sin(rwClockAngle),
                        -math.sin(rwElAngle) * math.cos(rwClockAngle), math.cos(rwElAngle)])

    rwClockAngle += 180.0 * math.pi / 180.0
    RWAJsList.extend([wheelJs])
    RWAGsMatrix.extend([math.sin(rwElAngle) * math.sin(rwClockAngle),
                        math.sin(rwElAngle) * math.cos(rwClockAngle), -math.cos(rwElAngle)])

    rwClockAngle -= 90.0 * math.pi / 180.0
    RWAJsList.extend([wheelJs])
    RWAGsMatrix.extend([math.sin(rwElAngle) * math.sin(rwClockAngle),
                        -math.sin(rwElAngle) * math.cos(rwClockAngle), math.cos(rwElAngle)])
    return RWAGsMatrix, RWAJsList


def CreateRWAClass():
    RWAGsMatrix, RWAJsList = create_rw_lists()
    rwClass = fswMessages.RWConstellationFswMsg()
    rwPointerList = list()
    rwClass.numRW = 4
    i = 0
    while i < 4:
        rwPointer = fswMessages.RWConfigElementFswMsg()
        rwPointer.gsHat_B = RWAGsMatrix[i * 3:i * 3 + 3]
        rwPointer.Js = RWAJsList[i]
        rwPointer.uMax = 0.2
        rwPointerList.append(rwPointer)
        i += 1
    rwClass.reactionWheels = rwPointerList
    return rwClass


def CreateRWAClassDyn():
    RWAGsMatrix, RWAJsList = create_rw_lists()
    rwConfigData = fswMessages.RWArrayConfigFswMsg()
    gsList = np.zeros(3 * fswMessages.MAX_EFF_CNT)
    gsList[0:3 * 3 + 3] = RWAGsMatrix[0:3 * 3 + 3]
    rwConfigData.GsMatrix_B = gsList
    jsList = np.zeros(fswMessages.MAX_EFF_CNT)
    jsList[0:4] = RWAJsList[0:4]
    rwConfigData.JsList = jsList
    rwConfigData.numRW = 4
    torqueMax = np.zeros(fswMessages.MAX_EFF_CNT)
    torqueMax[0:4] = [0.2] * 4
    rwConfigData.uMax = torqueMax
    return rwConfigData
