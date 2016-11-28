import sys, os, inspect
import EMMModuleParse as dataParser

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import AVSSim
import alg_contain
import sim_model
import numpy as np


def parseSimAlgorithms(TheSim, taskActivityDir, outputCFileName, str_ConfigData,
    simTag='TheSim'):
    def areTasksInSimTaskList(taskActivityDir, TheSim):
        print 'TASKS BEING PARSED: '
        taskIdxDir = {}
        l = len(TheSim.TaskList)
        for i_task in range(0, l):
            taskName = TheSim.TaskList[i_task].Name
            if taskName in taskActivityDir.keys():
                taskIdxDir[i_task] = str(taskActivityDir[taskName])
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
        for k, v in globalAlgUpdate.items():
            void_source += '\t' + 'if (data->' + v[0] + '){' + '\n'
            void_source += '\t\t' + k + '(data, callTime);' + '\n'
            void_source += '\t' + '}' + '\n'
        void_source += '}' + '\n'
        theVoidList.append(void_header)
        theAlgList.append(void_source)

    # This function creates two lists for the activity flag variables: declaration (header) and initialization (source)
    def writeTaskActivityVars(globalAlgUpdate, varType):
        theTaskActivity_declareList = []
        theTaskActivity_initList = []
        for k, v in globalAlgUpdate.items():
            declare_str = varType + ' ' + v[0] + ';' + '\n'
            init_str = 'data->' + v[0] + ' = ' + v[1] + ';' + '\n'
            theTaskActivity_declareList.append(declare_str)
            theTaskActivity_initList.append(init_str)
        return (theTaskActivity_declareList, theTaskActivity_initList)

    # This function looks for the path of the required header files
    def findFilePath(file):
        ADCSPath = path + '/../ADCSAlgorithms/'
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
    globalAlgUpdate = {} #
    theConfigDataList = []
    theAlgList = [] # global list for all source algorithms
    theVoidList = [] # global list for all header void function definitions.
    theHeadersList = [] # global list for all the module headers
    ConfigData = '(' + str_ConfigData + ' *data)'
    ConfigData_callTime = '(' + str_ConfigData + ' *data, uint64_t callTime)'


    taskIdxDir = areTasksInSimTaskList(taskActivityDir, TheSim)
    for i_task in taskIdxDir:
        task = TheSim.TaskList[i_task]
        taskActivity = taskIdxDir[i_task]
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
        globalAlgUpdate[algNameUpdate] = (algNameUpdateTaskActivity, taskActivity)
        algNameReset = task.Name + '_Reset'
        writeTaskAlgs(algNameUpdate + ConfigData_callTime, allAlgUpdate, theVoidList, theAlgList)
        if (allAlgReset): # check if there are any reset methods in the task models
            writeTaskAlgs(algNameReset + ConfigData_callTime, allAlgReset, theVoidList, theAlgList)
            for reset in allAlgReset:
                if not(reset in globalAllAlgReset):
                    globalAllAlgReset.append(reset)
    algNameAllSelfInit = 'AllAlg_SelfInit'
    algNameAllCrossInit = 'AllAlg_CrossInit'
    algNameAllReset = 'AllAlg_Reset'
    taskNameUpdate = 'AllTasks_Update'
    algNameDataInit = 'DataInit'

    writeTaskAlgs(algNameAllSelfInit + ConfigData, allAlgSelfInit, theVoidList, theAlgList)
    writeTaskAlgs(algNameAllCrossInit  + ConfigData, allAlgCrossInit, theVoidList, theAlgList)
    writeTaskAlgs(algNameAllReset + ConfigData_callTime, globalAllAlgReset, theVoidList, theAlgList)
    writeUpdateTaskActivityAlg(taskNameUpdate  + ConfigData_callTime, globalAlgUpdate, theVoidList, theAlgList)
    varType = 'uint32_t'
    (theTaskActivity_declareList, theTaskActivity_initList) = writeTaskActivityVars(globalAlgUpdate, varType)

    # Open/Create C source&header files and swig file
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    localPath = os.path.dirname(os.path.abspath(filename))
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
    defName = 'EMM_FSW_AUTOCODE_'
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

def defaultAVSSimTasks(taskActivityDir):
    taskActivityDir["initOnlyTask"] = 1
    taskActivityDir["sunSafeFSWTask"] = 1
    taskActivityDir["sunPointTask"] = 1
    taskActivityDir["earthPointTask"] = 1
    taskActivityDir["marsPointTask"] = 1
    taskActivityDir["vehicleAttMnvrFSWTask"] = 1
    taskActivityDir["vehicleDVPrepFSWTask"] = 1
    taskActivityDir["vehicleDVMnvrFSWTask"] = 1
    taskActivityDir["RWADesatTask"] = 1
    taskActivityDir["thrForceMappingTask"] = 1
    taskActivityDir["thrFiringSchmittTask"] = 1
    taskActivityDir["sensorProcessing"] = 1
    taskActivityDir["inertial3DPointTask"] = 1
    taskActivityDir["hillPointTask"] = 1
    taskActivityDir["velocityPointTask"] = 1
    taskActivityDir["celTwoBodyPointTask"] = 1
    taskActivityDir["rasterMnvrTask"] = 1
    taskActivityDir["initOnlyTask"] = 1
    taskActivityDir["eulerRotationTask"] = 1
    taskActivityDir["inertial3DSpinTask"] = 1
    taskActivityDir["attitudeControlMnvrTask"] = 1
    taskActivityDir["feedbackControlMnvrTask"] = 1
    taskActivityDir["attitudeControlMnvrTask"] = 1
    taskActivityDir["simpleRWControlTask"] = 1
    return taskActivityDir

if __name__ == "__main__":
    TheAVSSim = AVSSim.AVSSim()
    taskActivityDir = defaultAVSSimTasks({})
    outputFileName = 'EMM_FSW_Autocode'
    str_ConfigData = 'EMMConfigData'
    parseSimAlgorithms(TheAVSSim, taskActivityDir, outputFileName, str_ConfigData)


#TheAVSSim = AVSSim.AVSSim()
#taskIdxList = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
#taskIdxActivityDir = {}
#for taskIdx in taskIdxList:
#    taskIdxActivityDir[taskIdx] = str(1)
#outputCFileName = 'EMM_FSW_Autocode'
#str_ConfigData = 'EMMConfigData'
#parseSimAlgorithms(TheAVSSim, taskIdxActivityDir, outputCFileName, str_ConfigData)



