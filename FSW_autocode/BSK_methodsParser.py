import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')

import AVSSim
import alg_contain
import sim_model
import numpy as np

def parseSimAlgorithms(TheSim, taskIdxList, outputCFileName, str_ConfigData):
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

    # This function checks the method type of the input and returns the corresponding string.
    #   methodName: name of the model data algorithm
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
        if dict[modelTagKey][0] == addressVal:
            IDVal = dict[modelTagKey][1]
            return IDVal
        raise ValueError('Unable to find address match.')

    # This function progressively creates a list with all the void definitions that will go in the output header file
    # and the algorithms that will go in the output source file
    def writeTaskAlgs(str_ConfigData, algName, algList, theVoidList, theAlgList):
        ConfigData = '(' + str_ConfigData + ' *data)'
        void = 'void ' + algName + ConfigData
        void_header = void + ';\n'
        void_source = void + '\n{\n'
        for alg in algList:
            void_source += '\t' + alg + ';\n'
        void_source += '}\n'
        theVoidList.append(void_header)
        theAlgList.append(void_source)

    # This function appends a module's header string to the global headers list (only if it's not there)
    def createModuleHeaderName(module, headersList):
        moduleName = module[:len(module) / 2]
        if not(moduleName in headersList):
            header = '#include "' + moduleName + '.h"\n'
            headersList.append(header)

    # Model Wraps
    SelfInit_dict = {}  # dictionary D = {modelTag: SelfInit alg address, moduleID}
    CrossInit_dict = {}  # dictionary D = {modelTag: CrossInit alg address, moduleID}
    Update_dict = {}  # dictionary D = {modelTag: Update alg address, moduleID}
    Reset_dict = {}  # dictionary D = {modelTag: Reset alg address, moduleID}
    TheSimList = dir(TheSim)
    i = 0
    for elemName in TheSimList:
        elem = eval('TheSim.' + elemName)
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
    theAlgList = [] # global list for all source algorithms
    theVoidList = [] # global list for all header void function definitions.
    theHeadersList = [] # global list for all the module headers
    for i_task in taskIdxList:
        task = TheSim.TaskList[i_task]
        #print 'Task = ', task.Name
        allAlgUpdate = [] # local list for task models' Update algorithms
        allAlgReset = [] # local list for task model's Reset algorithms
        i_model = 0
        for model in task.TaskModels:
            key = getTaskModelKey(i_task, i_model)
            modelTag = NameReplaceList[key]
            module = model.__module__
            createModuleHeaderName(module, theHeadersList)
            sysMod = sys.modules[module]
            dirList = dir(sysMod)
            parsed_dirList = parseSwigVars(dirList)
            addressDict = evalParsedList(parsed_dirList, module)
            for k, v in addressDict.items():
                methodType = checkMethodType(k)
                dict = eval(methodType + '_dict')
                modelID = findAddressMatch(v, modelTag, dict)
                #print modelTag, modelID, k
                theString = k +'(&(data->' + modelTag + '), ' + str(modelID) + ')'
                theList = eval('allAlg' + methodType)
                if not(theString in theList):
                    theList.append(theString)
            i_model += 1
        algNameUpdate = task.Name + '_Update'
        algNameReset = task.Name + '_Reset'
        writeTaskAlgs(str_ConfigData, algNameUpdate, allAlgUpdate, theVoidList, theAlgList)
        if (allAlgReset): # check if there are any reset methods in the task models
            writeTaskAlgs(str_ConfigData, algNameReset, allAlgReset, theVoidList, theAlgList)
    algNameAllSelfInit = 'allAlg_SelfInit'
    algNameAllCrossInit = 'allAlg_CrossInit'
    writeTaskAlgs(str_ConfigData, algNameAllSelfInit, allAlgSelfInit, theVoidList, theAlgList)
    writeTaskAlgs(str_ConfigData, algNameAllCrossInit, allAlgCrossInit, theVoidList, theAlgList)

    # Open/Create C source and header files
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    localPath = os.path.dirname(os.path.abspath(filename))
    fileMode = 'w+'  # create file to be written if it doesn't exist
    # Write source file
    alg_source = open(localPath + '/' + outputCFileName + '.c', fileMode)  # source file
    alg_source.write('#include ' + outputCFileName + '.h' + '\n\n')
    for alg in theAlgList:
        alg_source.write(alg)
    alg_source.close()
    # Write header file
    alg_header = open(localPath + '/' + outputCFileName + '.h', fileMode)  # header file
    defName = '_SIM_ALGORITHMS_'
    alg_header.write('#ifndef ' + defName + '\n' + '#define ' + defName + '\n\n')
    for header in theHeadersList:
        alg_header.write(header)
    alg_header.write('\n' + '#ifdef ' + '__cplusplus' + '\n' + 'extern "C" {' + '\n' + '#endif' + '\n')
    for void in theVoidList:
        alg_header.write('\t')
        alg_header.write(void)
    alg_header.write('#ifdef ' + '__cplusplus' + '\n' + '}' + '\n' + '#endif')
    alg_header.close()



TheAVSSim = AVSSim.AVSSim()
taskIdxList = [2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 19, 20, 21, 22, 23, 24, 25, 26]
outputCFileName = 'EMMAlgorithms'
str_ConfigData = 'EMMConfigData'
parseSimAlgorithms(TheAVSSim, taskIdxList, outputCFileName, str_ConfigData)





