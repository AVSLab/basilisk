
'''This document contains technology controlled by the U.S. Export Administration
Regulations, 15 C.F.R. Parts 740-774 (EAR). Transfer, disclosure, or export to 
foreign persons without prior U.S. Government approval may be prohibited. 
Violations of these export laws and regulations are subject to severe civil 
and criminal penalties.
'''

import sys, os, inspect, re

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules')

# import AVSSim

# ----------------------------- METHODS ----------------------------- #
def matchDictValueToKey(dict,searchVal):
    return dict.keys()[dict.values().index(searchVal)]

def getDataTypeStr(typStr):
    typStr = typStr[(typStr.find("'")+1):len(typStr)]
    typStr = typStr[0:typStr.find("'")]
    if typStr.find(' *') >= 0:
        typStr = typStr[0:typStr.find(' *')]
    if typStr.find('int') >= 0:
        typStr = typStr[typStr.find('int'):(typStr.find('int')+3)]
    return typStr

def makeTaskModelString(i,j):
    return 'self.TaskList['+str(i)+'].TaskModels['+str(j)+']'

def autoCode():
    # main autocode sequence
    print TaskListIdxs
    for i in TaskListIdxs: # loop through TaskLists
        auxFile.write('\n\n'+'TaskList: '+str(i)+'\n') # print current Tasklist index to workfile

        for j in range(0, len(TaskList[i].TaskModels)): # loop through TaskModels in TaskList
                auxFile.write('\n'+'TaskModel: '+str(j)+'\n') # print current taskmodel index to workfile

                fieldStr = str(type(TaskList[i].TaskModels[j]).__name__)
                fieldStrInterface = matchDictValueToKey(NameReplace,makeTaskModelString(i,j))
                EMMInith.write('\t' + fieldStr + ' ' + fieldStrInterface + ';\n')

                varNameList = dir(TaskList[i].TaskModels[j]) # list all the variable names under current TaskModel

                for k in range(0, len(varNameList)): # loop through variables within current TaskModel
                    auxFile.write('\n'+str(varNameList[k])+'\n') # print name of current element of taskmodel to workfile

                    varName = varNameList[k]
                    varValue = getattr(TaskList[i].TaskModels[j], varName) # get value of current element of TaskModel

                    print varName

                    # innerAutocode(varName,varValue,fieldStrInterface)
                    varType = type(varValue).__name__ # datatype of value of current element
                    varTypeFull = str(type(varValue))

                    auxFile.write(varTypeFull+'\n') # print datatype to workfile
                    auxFile.write(str(varValue)+'\n') # print value of current element to workfile

                    # __.__ and this
                    if (varName[0:2] == '__') or (varName[0:4] == 'this'):
                        continue # skip __.__ and this

                    # class
                    elif varTypeFull[1:6] == 'class':
                        # innerAutocode(varName,varValue,'WhatGoesHere')
                        continue # skip classes

                    # character array
                    elif varType == 'str':
                        dest = ConfigDataStr + fieldStrInterface + '.' + str(varName)
                        EMMInitc.write('\t'+'strcpy(' + dest + ',' + '"'+str(varValue)+'"' + ')'+';\n')

                    # array (SwigPyObject) or method (instancemethod)
                    elif varType == 'SwigPyObject' or varType == 'instancemethod':
                        typeStr = varValue.__str__()

                        if ((typeStr.find('void') >= 0) or (typeStr.find('AlgContain') >= 0)):
                            continue
                        else:
                            typeStr = getDataTypeStr(typeStr)

                        arr = AVSSim.SimulationBaseClass.getCArray(typeStr,varValue,arrMaxLen)

                        for l in range(0,arrMaxLen):
                            EMMInitc.write('\t' + ConfigDataStr + fieldStrInterface + '.' + str(varName) + '[' + str(l) + '] = ' + str(arr[l])+';\n')

                    # non-array variable
                    else:
                        EMMInitc.write('\t' + ConfigDataStr + fieldStrInterface + '.')
                        EMMInitc.write(str(varName)) # name of the variable
                        EMMInitc.write(' = '+str(varValue)+';\n') # value of the variable

    EMMInitc.write('}')
    EMMInith.write('}EMMConfigData;')



    print("Goin' to Mars2")


# ----------------------------- MAIN ----------------------------- #
if __name__ == "__main__":

    print 'started'

    # TheAVSSim = AVSSim.AVSSim()

    # constants and such
    ConfigDataStr = 'ConfigData->' # name of the main struct
    arrMaxLen = 3
    NameReplace = TheAVSSim.NameReplace
    TaskList = TheAVSSim.TaskList
    TaskListIdxs = [10, 12, 13, 14, 15, 19, 20, 22, 23, 24, 25, 21, 26]


    # open the files for writing
    auxFile = open(path + '/../FSW_autocode/autocode_aux.txt', 'w') # auxFile is useful for debugging
    EMMInitc = open(path + '/../FSW_autocode/EMMInit.c', 'w') # .c file
    EMMInith = open(path + '/../FSW_autocode/EMMInit.h', 'w') # .h file

    # print initialization text to the files
    EMMInitc.write('void dataInitialization(EMMConfigData *ConfigData) {\n')
    EMMInith.write('typedef struct {\n')

    # make sure all TaskModels have a NameReplace, since this is used for unique naming
    for i in range(0,len(TaskList)):
        for j in range(0,len(TaskList[i].TaskModels)):
            tmStr = makeTaskModelString(i,j)
            if not tmStr in NameReplace.values():
                NameReplace['TaskList_'+str(i)+'_TaskModel_'+str(j)] = tmStr

    autoCode()

    auxFile.close() # close auxFile
    EMMInitc.close() # close EMMInitc file
    EMMInith.close() # close EMMInith file

    print 'stopped'