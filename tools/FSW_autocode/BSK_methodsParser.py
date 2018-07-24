''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
import sys, os, inspect
import BSKModuleParse as dataParser

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.simulation import alg_contain
from Basilisk.simulation import sim_model
import numpy as np


def parseSimAlgorithms(TheSim, taskActivityDir, outputCFileName, str_ConfigData,
    simTag='TheSim', localPath = os.path.dirname(os.path.abspath(filename))):
    def areTasksInSimTaskList(taskActivityDir, TheSim):
        print 'TASKS BEING PARSED: '
        taskIdxDir = []
        taskOrderedList = []
        procLength = len(TheSim.TotalSim.processList)
        for procIdx in range(procLength):
            locProcList = []
            for i_task in range(len(TheSim.TotalSim.processList[procIdx].processTasks)):
                theTask = TheSim.TotalSim.processList[procIdx].processTasks[i_task]
                taskFound = False
                for ordIdx in range(len(locProcList)):
                    locTask = locProcList[ordIdx]
                    # locTask[0] = taskName
                    # locTask[1] = taskPriority
                    if theTask.taskPriority > locTask[1]:
                        locProcList.insert(ordIdx,
                            [theTask.TaskPtr.TaskName, theTask.taskPriority])
                        taskFound = True
                        break
                if taskFound != True:
                    locProcList.append([theTask.TaskPtr.TaskName, theTask.taskPriority, theTask])
            taskOrderedList.extend(locProcList)

        for i_task in range(0, len(taskOrderedList)):
            # taskOrderedList[i_task][0] = taskName
            # taskOrderedList[i_task][1] = taskPriority
            # taskOrderedList[i_task][2] = theTask
            taskName = taskOrderedList[i_task][0]
            if taskName in taskActivityDir.keys():
                idxUse = getTaskIndex(TheSim, taskOrderedList[i_task][2].TaskPtr)
                taskIdxDir.append(idxUse)
                print i_task, taskName
        return taskIdxDir

    # First Parsing Method:
    # Get rid of all the variables that come after a built-in. The methods SelfInit, CrossInit, Update and Restart
    # will always come first, and so will do the variables that are capitalized (this is based on how "dir()" command
    # works in the Python interpreter).
    # Returns a reduced array.
    def parseSwigVars(list):
        parsed_array = np.array([])
        length = len(list)
        i = 0
        while i < length:
            if list[i][0] != '_':
                parsed_array = np.append(parsed_array, list[i])
                i += 1
            else:
                break
        return parsed_array

    # Second Parsing Method:
    # Collects all the SwigPyObjects present in the list. Only the methods SelfInit_(...), CrossInit_(...),
    # Update_(...) and Restart_(...) are wrapped by Swig in the .i files. Therefore they are the only SwigPyObjects.
    # Returns a dictionary D = {method, address}
    def evalParsedList(list, module):
        addressDict = {}
        for methodName in list:
            methodObject = eval('sys.modules["' + module + '"].' + methodName)
            if type(methodObject).__name__ == "SwigPyObject":
                methodAddress = sim_model.getObjectAddress(methodObject)
                addressDict[methodName] = int(methodAddress)
        return addressDict

    # This function checks the method type of the input and returns the corresponding strings.
    #   methodName: name of the model data algorithm
    def checkMethodType(methodName):
        str_selfInit = 'SelfInit'
        str_crossInit = 'CrossInit'
        str_update = 'Update'
        str_reset = 'Reset'

        str_blank = '' # the methods SelfInit and CrossInit don't need the callTime parameter
        str_callTime = ', callTime' # the methods Reset and Update need an extra parameter for callTine

        if methodName[0:len(str_selfInit)] == str_selfInit:
            return (str_selfInit, str_blank)
        elif methodName[0:len(str_crossInit)] == str_crossInit:
            return (str_crossInit, str_blank)
        elif methodName[0:len(str_update)] == str_update:
            return (str_update, str_callTime)
        elif methodName[0:len(str_reset)] == str_reset:
            return (str_reset, str_callTime)
        else:
            raise ValueError('Cannot recognize the method. Parse better.')

    # This function returns the key name of the NameReplace dictionary according to the index of the task
    # and the index of the model inside the task
    def getTaskModelKey(i_task, i_model):
        key = 'self.TaskList[' + str(i_task) + '].TaskModels[' + str(i_model) + ']'
        return key


    # This function returns the key name of the NameReplace dictionary according to the index of the task
    # and the index of the model inside the task
    def getTaskIndex(theSim,taskUse):
        j=0
        for taskPy in theSim.TaskList:
            if taskUse.TaskName == taskPy.Name:
                return j
            j+=1
        return -1

    # This function makes sure that each algorithm in a data model is matched with the proper algorithm in
    # the corresponding model wrap. If there's a match, returns the ID of the model. Otherwise, an ugly
    # error is raised and the whole program quits.
    # addressVal: address of the model data algorithm
    # modelTagKey: modelTag = key with which to search in "dict"
    # dict: wrap algorithms' address dictionary in which to look for a match
    #   dict[modelTagKey][0] = model wrap algorithm address
    #   dict[modelTagKey][1] = model ID
    def findAddressMatch(addressVal, modelTagKey, dict):
        try:
            address = dict[modelTagKey][0]
            if address == addressVal:
                IDVal = dict[modelTagKey][1]
                return IDVal
        except:
            raise ValueError(str(modelTagKey) + ' is not wrapping all the existing algorithms in '
                                                'the corresponding data model. Fix it.')

    # This function progressively creates a list with all the void definitions that will go in the output header file
    # and the algorithms names that will go in the output source file for SelfInit, CrossInit and Reset methods
    def writeTaskAlgs(algName, algList, theVoidList, theAlgList):
        void = 'void ' + algName
        void_header = void + ';\n'
        void_source = void + '\n{\n'
        for alg in algList:
            void_source += '\t' + alg + ';\n'
        void_source += '}\n'
        theVoidList.append(void_header)
        theAlgList.append(void_source)

    # This function progressively creates a list with all the void definitions that will go in the output header file
    # and the algorithms names that will go in the output source file for Update methods.
    # It adds an if-check to learn about the task activity flag. Only tasks that are active should be Updated.
    def writeUpdateTaskActivityAlg(algName, globalAlgUpdate, theVoidList, theAlgList):
        void = 'void ' + algName
        void_header = void + ';\n'
        void_source = void + '\n{\n'
        for updateElem in globalAlgUpdate:
            void_source += '\t' + 'if (data->' + updateElem[1][0] + '){' + '\n'
            void_source += '\t\t' + updateElem[0] + '(data, callTime);' + '\n'
            void_source += '\t' + '}' + '\n'
        void_source += '}' + '\n'
        theVoidList.append(void_header)
        theAlgList.append(void_source)

    # This function creates two lists for the activity flag variables: declaration (header) and initialization (source)
    def writeTaskActivityVars(globalAlgUpdate, varType):
        theTaskActivity_declareList = []
        theTaskActivity_initList = []
        for updateElem in globalAlgUpdate:
            # updateElem[0] = algNameUpdate
            # updateElem[1] = [algNameTaskActivity, boolIsTaskActive]
            declare_str = varType + ' ' + updateElem[1][0] + ';' + '\n'
            init_str = 'data->' + updateElem[1][0] + ' = ' + updateElem[1][1] + ';' + '\n'
            theTaskActivity_declareList.append(declare_str)
            theTaskActivity_initList.append(init_str)
        return (theTaskActivity_declareList, theTaskActivity_initList)

    # This function looks for the path of the required header files
    def findFilePath(file):
        ADCSPath = path + '/../fswAlgorithms/'
        for dirpath, subdirs, files in os.walk(ADCSPath):
            for x in files:
                if x == file:
                    relDir = os.path.relpath(dirpath, path)
                    filePath = os.path.join(relDir, file)
                    return filePath
    # This function appends a module's header string to the global headers list (only if it's not there)
    def createModuleHeaderName(module, headersList):
        moduleName = module[:len(module) / 2] + '.h'
        headerPath = findFilePath(moduleName)
        if(headerPath == None):
           return
        header = '#include "' + headerPath + '"\n'
        if not(header in headersList):
            headersList.append(header)

    # This function creates a list with the declaration of all config data structures
    def createConfigDataHeader(tag, configData, configlist):
        string = configData + ' ' + tag + ';' + '\n'
        if not (string in configlist):
            configlist.append(string)


    # Model Wraps
    SelfInit_dict = {}  # dictionary D = {modelTag: SelfInit alg address, moduleID}
    CrossInit_dict = {}  # dictionary D = {modelTag: CrossInit alg address, moduleID}
    Update_dict = {}  # dictionary D = {modelTag: Update alg address, moduleID}
    Reset_dict = {}  # dictionary D = {modelTag: Reset alg address, moduleID}
    TheSimList = dir(eval(simTag))
    i = 0
    for elemName in TheSimList:
        elem = eval(simTag + '.' + elemName)
        if type(elem) == alg_contain.AlgContain:
            SelfInit_dict[elem.ModelTag] = (int(elem.getSelfInitAddress()), i)
            CrossInit_dict[elem.ModelTag] = (int(elem.getCrossInitAddress()), i)
            Update_dict[elem.ModelTag] = (int(elem.getUpdateAddress()), i)
            hasResetAddress = int(elem.getResetAddress())
            if (hasResetAddress):
                Reset_dict[elem.ModelTag] = (hasResetAddress, i)
            i += 1

    # Model Data
    NameReplaceList = TheSim.NameReplace
    allAlgSelfInit = [] # global list for all models' SelfInit algorithms
    allAlgCrossInit = [] # global list for all models' CrossInit algorithms
    globalAllAlgReset = [] # global list for all models' Reset algorithms
    globalAlgUpdate = [] #
    theConfigDataList = []
    theAlgList = [] # global list for all source algorithms
    theVoidList = [] # global list for all header void function definitions.
    theHeadersList = [] # global list for all the module headers
    ConfigData = '(' + str_ConfigData + ' *data)'
    ConfigData_callTime = '(' + str_ConfigData + ' *data, uint64_t callTime)'


    taskIdxDir = areTasksInSimTaskList(taskActivityDir, TheSim)
    for i_task in taskIdxDir:
        task = TheSim.TaskList[i_task]
        isTaskActive = taskActivityDir[task.Name]
        allAlgUpdate = [] # local list for task models' Update algorithms
        allAlgReset = [] # local list for task model's Reset algorithms
        i_model = 0
        for model in task.TaskModels:
            key = getTaskModelKey(i_task, i_model)
            modelTag = NameReplaceList[key]
            module = model.__module__
            modelConfigDataName = str(type(model).__name__)
            createConfigDataHeader(modelTag, modelConfigDataName, theConfigDataList)
            createModuleHeaderName(module, theHeadersList)
            sysMod = sys.modules[module]
            dirList = dir(sysMod)
            parsed_dirList = parseSwigVars(dirList)
            addressDict = evalParsedList(parsed_dirList, module)
            for k, v in addressDict.items():
                (methodType, methodCallTime) = checkMethodType(k)
                dictUse = eval(methodType + '_dict')
                modelID = findAddressMatch(v, modelTag, dictUse)
                theString = k +'(&(data->' + modelTag + ')' + methodCallTime + ', ' + str(modelID) + ')'
                theList = eval('allAlg' + methodType)
                if not(theString in theList):
                    theList.append(theString)
            i_model += 1

        algNameUpdate = task.Name + '_Update'
        algNameUpdateTaskActivity = task.Name + '_isActive'
        #globalAlgUpdate.append([algNameUpdate, (algNameUpdateTaskActivity, str(0))])
        globalAlgUpdate.append([algNameUpdate, (algNameUpdateTaskActivity,isTaskActive)])
        algNameReset = task.Name + '_Reset'
        writeTaskAlgs(algNameUpdate + ConfigData_callTime, allAlgUpdate, theVoidList, theAlgList)
        if (allAlgReset): # check if there are any reset methods in the task models
            writeTaskAlgs(algNameReset + ConfigData_callTime, allAlgReset, theVoidList, theAlgList)
            for reset in allAlgReset:
                if not(reset in globalAllAlgReset):
                    globalAllAlgReset.append(reset)
    algNameAllSelfInit = str_ConfigData + '_AllAlg_SelfInit'
    algNameAllCrossInit = str_ConfigData + '_AllAlg_CrossInit'
    algNameAllReset = str_ConfigData + '_AllAlg_Reset'
    taskNameUpdate = str_ConfigData + '_AllTasks_Update'
    algNameDataInit = str_ConfigData + '_DataInit'

    writeTaskAlgs(algNameAllSelfInit + ConfigData, allAlgSelfInit, theVoidList, theAlgList)
    writeTaskAlgs(algNameAllCrossInit  + ConfigData, allAlgCrossInit, theVoidList, theAlgList)
    writeTaskAlgs(algNameAllReset + ConfigData_callTime, globalAllAlgReset, theVoidList, theAlgList)
    writeUpdateTaskActivityAlg(taskNameUpdate  + ConfigData_callTime, globalAlgUpdate, theVoidList, theAlgList)
    varType = 'uint32_t'
    (theTaskActivity_declareList, theTaskActivity_initList) = writeTaskActivityVars(globalAlgUpdate, varType)

    # Open/Create C source&header files and swig file
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    fileMode = 'w+'  # create file to be written if it doesn't exist
    alg_source = open(localPath + '/' + outputCFileName + '.c', fileMode)  # source file
    alg_header = open(localPath + '/' + outputCFileName + '.h', fileMode)  # header file
    alg_swig = open(localPath + '/' + outputCFileName + '.i', fileMode)  # header file

    # Write source file
    alg_source.write('#include "' + outputCFileName + '.h"' + '\n\n')
    for alg in theAlgList:
        alg_source.write(alg)
    alg_source.write('\n')
    theDataVoid = 'void ' + algNameDataInit + '(' + str_ConfigData + ' *data)'
    alg_source.write(theDataVoid + '{\n')
    for init in theTaskActivity_initList:
        alg_source.write('\t')
        alg_source.write(init)
    dataParser.ParseConfigData(TheSim, taskIdxDir, 'data->', alg_source)
    alg_source.write('}')

    # Write header file
    ## begin header file
    defName = 'AVS_FSW_AUTOCODE_'
    alg_header.write('#ifndef ' + defName + '\n' + '#define ' + defName + '\n\n')
    ## define auto-code init data
    for header in theHeadersList:
        alg_header.write(header)
    alg_header.write('\n' + 'typedef struct{' + '\n')
    for configData in theConfigDataList:
        alg_header.write('\t')
        alg_header.write(configData)
    for declare in theTaskActivity_declareList:
        alg_header.write('\t')
        alg_header.write(declare)
    alg_header.write('}' + str_ConfigData +';' + '\n')
    ## define auto-code algorithms
    alg_header.write('\n' + '#ifdef ' + '__cplusplus' + '\n' + 'extern "C" {' + '\n' + '#endif' + '\n')
    for void in theVoidList:
        alg_header.write('\t')
        alg_header.write(void)
    alg_header.write('\t' + theDataVoid + ';' + '\n')
    alg_header.write('#ifdef ' + '__cplusplus' + '\n' + '}' + '\n' + '#endif')
    alg_header.write('\n\n' + '#endif')

    # Write swig file
    def writeSwigAlgCode(algNames, swigFile):
        endL = ';\n'
        for name in algNames:
            constant_str = '%constant void ' + name + endL
            ignore_str = '%ignore ' + name + endL
            swigFile.write(constant_str + ignore_str)
        return
    ## begin swig code
    alg_swig.write('%module ' + outputCFileName + '\n' + '%{' + '\n')
    alg_swig.write('\t' + '#include "' + outputCFileName + '.h"' + '\n' + '%}' + '\n\n')
    alg_swig.write('%include "swig_conly_data.i"' + '\n\n')
    ## wrap C auto-code algorithms
    configData_paramsDefault = '(void*, unit64_t)'
    configData_paramsCallTime =  '(void*, unit64_t, uint64_t)'
    algNames = [
        algNameDataInit +configData_paramsDefault,
        algNameAllSelfInit + configData_paramsDefault,
        algNameAllCrossInit + configData_paramsDefault,
        algNameAllReset + configData_paramsCallTime,
        taskNameUpdate + configData_paramsCallTime
    ]
    writeSwigAlgCode(algNames, alg_swig)
    ## end swig code
    alg_swig.write('%include "'+ outputCFileName + '.h"' + '\n\n')
    alg_swig.write('%pythoncode %{' + '\n' + 'import sys' + '\n' +
                   'protectAllClasses(sys.modules[__name__])' + '\n' + '%}')

    # Close C source&header files and swig file
    alg_source.close()
    alg_header.close()
    alg_swig.close()



# ---------------------------------- MAIN ---------------------------------- #

def defaultAVSSimTasks(boolActive):
    taskActivityDir = {}
    taskActivityDir["initOnlyTask"] = boolActive
    taskActivityDir["sunSafeFSWTask"] = boolActive
    taskActivityDir["sunPointTask"] = boolActive
    taskActivityDir["earthPointTask"] = boolActive
    taskActivityDir["marsPointTask"] = boolActive
    taskActivityDir["vehicleAttMnvrFSWTask"] = boolActive
    taskActivityDir["vehicleDVPrepFSWTask"] = boolActive
    taskActivityDir["vehicleDVMnvrFSWTask"] = boolActive
    taskActivityDir["RWADesatTask"] = boolActive
    taskActivityDir["thrForceMappingTask"] = boolActive
    taskActivityDir["thrFiringSchmittTask"] = boolActive
    taskActivityDir["sensorProcessing"] = boolActive
    taskActivityDir["inertial3DPointTask"] = boolActive
    taskActivityDir["hillPointTask"] = boolActive
    taskActivityDir["velocityPointTask"] = boolActive
    taskActivityDir["celTwoBodyPointTask"] = boolActive
    taskActivityDir["rasterMnvrTask"] = boolActive
    taskActivityDir["initOnlyTask"] = boolActive
    taskActivityDir["eulerRotationTask"] = boolActive
    taskActivityDir["inertial3DSpinTask"] = boolActive
    taskActivityDir["attitudeControlMnvrTask"] = boolActive
    taskActivityDir["feedbackControlMnvrTask"] = boolActive
    taskActivityDir["attitudeControlMnvrTask"] = boolActive
    taskActivityDir["simpleRWControlTask"] = boolActive
    return taskActivityDir

if __name__ == "__main__":
    TheAVSSim = AVSSim.AVSSim()
    taskActivityDir = defaultAVSSimTasks(str(0))
    outputFileName = 'AVS_FSW_Autocode'
    str_ConfigData = 'AVSConfigData'
    parseSimAlgorithms(TheAVSSim, taskActivityDir, outputFileName, str_ConfigData)


#TheAVSSim = AVSSim.AVSSim()
#taskIdxList = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
#taskIdxActivityDir = {}
#for taskIdx in taskIdxList:
#    taskIdxActivityDir[taskIdx] = str(1)
#outputCFileName = 'AVS_FSW_Autocode'
#str_ConfigData = 'AVSConfigData'
#parseSimAlgorithms(TheAVSSim, taskIdxActivityDir, outputCFileName, str_ConfigData)
