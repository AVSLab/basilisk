import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')

import AVSSim
import alg_contain
import sim_model
import numpy as np
np.set_printoptions(precision=9)

# ----------------------------- METHODS ----------------------------- #
def parseSwigVars(list):
    parsed_array = np.array([])
    length = len(list)
    i = 0
    while i < length:
        if list[i][0]!='_':
            parsed_array = np.append(parsed_array, list[i])
            i += 1
        else:
            break
    return parsed_array

def evalParsedList(list):
    addressDict = {}
    for methodName in list:
        methodObject = eval('sys.modules["' + module + '"].' + methodName)
        if type(methodObject).__name__ == "SwigPyObject":
            methodAddress = sim_model.getObjectAddress(methodObject)
            addressDict[methodName] = int(methodAddress)
    return addressDict

def checkMethodType(methodName):
    str_selfInit = 'SelfInit'
    str_crossInit = 'CrossInit'
    str_update = 'Update'
    str_reset = 'Reset'
    if methodName[0:len(str_selfInit)] == str_selfInit:
        return str_selfInit
    elif methodName[0:len(str_crossInit)] == str_crossInit:
        return str_crossInit
    elif methodName[0:len(str_update)] == str_update:
        return str_update
    elif methodName[0:len(str_reset)] == str_reset:
        return str_reset
    else:
        raise ValueError('Cannot recognize the method. Parse better.')

def findAddressMatch(methodType, address):
    dict = eval(methodType + '_dict')
    for k, v in dict.items():
        if v == address:
            return k
    raise ValueError('Unable to find address match.')

def writeCFiles(sourceFile, headerFile, taskName, str_alg):
    str_EMMData = '(EMMConfigData *data)'
    algList = eval(str_alg)
    if len(algList) > 0:
        void_def = '\n void ' + taskName + '_' + str_alg + str_EMMData
        headerFile.write(void_def + ';\n')
        sourceFile.write('\n')
        sourceFile.write(void_def)
        sourceFile.write('\n { \n')
        for algName in algList:
            sourceFile.write('\t')
            sourceFile.write(algName)
            sourceFile.write('\n')
        sourceFile.write('}')
        sourceFile.write('\n')

def printWrap_allDicts():
    print 'selfInit_ModelDict = ', SelfInit_dict
    print 'crossInit_ModelDict = ', CrossInit_dict
    print 'update_ModelDict = ', Update_dict
    print 'reset_ModelDict = ', Reset_dict
    print '\n'

def printTaskData_AllAlg():
    print 'allAlgSelfInit = ', allAlgSelfInit
    print 'allAlgCrossInit = ', allAlgCrossInit
    print 'allAlgUpdate = ', allAlgUpdate
    print 'allAlgReset = ', allAlgReset
    print '\n'

# ----------------------------- MAIN ----------------------------- #
if __name__ == "__main__":
    alg_source = open(path + '/../../Basilisk/FSW_autocode/EMMAlgorithms.c', 'w+')
    alg_header = open(path + '/../../Basilisk/FSW_autocode/EMMAlgorithms.h', 'w+')

    TheAVSSim = AVSSim.AVSSim()

    # Model Wraps
    SelfInit_dict = {}
    CrossInit_dict = {}
    Update_dict = {}
    Reset_dict = {}
    TheAVSList = dir(TheAVSSim)
    for elemName in TheAVSList:
        elem = eval('TheAVSSim.' + elemName)
        if type(elem) == alg_contain.AlgContain:
            SelfInit_dict[elem.ModelTag] = int(elem.getSelfInitAddress())
            CrossInit_dict[elem.ModelTag] = int(elem.getCrossInitAddress())
            Update_dict[elem.ModelTag] = int(elem.getUpdateAddress())
            if (elem.getResetAddress()):
                Reset_dict[elem.ModelTag] = int(elem.getResetAddress())
                reset = int(elem.getResetAddress())
    printWrap_allDicts()

    # Model Data
    dict_TagID = {}
    ID_counter = 0
    taskIdxList = [10, 12, 13, 14, 15, 19, 20, 21, 22, 23, 24, 25, 26]
    #FSW Tasks Out: 11. attitudeNav, 16. singleAxisSpinTask, 17. orbitAxisSpinTask, 18. axisScanTask
    for i_task in taskIdxList:
        allAlgSelfInit = []
        allAlgCrossInit = []
        allAlgUpdate = []
        allAlgReset = []
        print 'Task Index = ', i_task
        task = TheAVSSim.TaskList[i_task]
        print 'Task Name = ', task.Name
        for model in task.TaskModels:
            module = model.__module__
            print 'Module = ', module
            sysMod = sys.modules[module]
            dirList = dir(sysMod)
            parsed_dirList = parseSwigVars(dirList)
            addressDict = evalParsedList(parsed_dirList)
            for k, v in addressDict.items():
                methodType = checkMethodType(k)
                modelTag = findAddressMatch(methodType, v)
                try:
                    modelID = dict_TagID[modelTag]
                except:
                    dict_TagID[modelTag] = ID_counter
                    modelID = ID_counter
                    ID_counter += 1
                theString = k +'(&(data->' + modelTag + '), ' + str(modelID) + ')'
                theList = eval('allAlg' + methodType)
                theList.append(theString)
        printTaskData_AllAlg()

        # Output data in C source and header files
        str_algSelfInit = 'allAlgSelfInit'
        str_algCrossInit = 'allAlgCrossInit'
        str_algUpdate = 'allAlgUpdate'
        str_algReset = 'allAlgReset'
        writeCFiles(alg_source, alg_header, task.Name, str_algSelfInit)
        writeCFiles(alg_source, alg_header, task.Name, str_algCrossInit)
        writeCFiles(alg_source, alg_header, task.Name, str_algUpdate)
        writeCFiles(alg_source, alg_header, task.Name, str_algReset)

    alg_source.close()
    alg_header.close()









