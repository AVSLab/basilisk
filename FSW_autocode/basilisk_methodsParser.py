import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')

import AVSSim
import alg_contain
import sim_model
import numpy as np
np.set_printoptions(precision=9)



def parseSimAlgorithms(TheSim, taskIdxList, cFileName, str_ConfigData):
    # This function prints dicts D = {ModelTag, alg. address} of all the model wraps
    def printWrap_allDicts():
        print 'selfInit_ModelDict = ', SelfInit_dict
        # for k, v in SelfInit_dict.items():
        #     print k, v
        print 'crossInit_ModelDict = ', CrossInit_dict
        print 'update_ModelDict = ', Update_dict
        print 'reset_ModelDict = ', Reset_dict
        print '\n'
    # This functions prints dicts D = {alg. name, moduleID} of all the model data
    def printTaskData_AllAlg():
        print 'allAlgUpdate = ', allAlgUpdate
        print 'allAlgReset = ', allAlgReset
        print '\n'

    # First Parsing Method:
    # Get rid of all the variables that come after a built-in. The methods SelfInit, CrossInit, Update and Restart
    # will always come first, and so will the variables that are capitalized (this is based on how "dir()" command
    # works in the Python interpreter).
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
    # Collect all the SwigPyObjects present in the list. Only the methods SelfInit, CrossInit, Update and Restart
    # are wrapped by Swig in the .i files. Therefore they are the only SwigPyObjects,
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

    # This function looks for an address match between model data algorithm and model wrap algorithm.
    #   address: model data algorithm address
    #   dict: dictionary of model wraps algorithm addresses
    def findAddressMatch(address, dict):
        for k, v in dict.items():
            if v == address:
                return k
        raise ValueError('Unable to find address match.')

    # This function outputs the algorithms belonging to a specific task in a C source and header files.
    #   sourceFile: output c file
    #   headerFile: output h file
    #   str_ConfigData, taskName, str_algList: string-type inputs used to create unique method names in the C files
    #   algList: list containing all the model algorithms to be printed inside the main functions
    def writeCFiles(sourceFile, headerFile, str_ConfigData, taskName, str_algList, algList):
        ConfigData = '(' + str_ConfigData + ' *data)'
        if len(algList) > 0:
            void_def = 'void ' + taskName + '_' + str_algList + ConfigData
            headerFile.write(void_def + ';\n')
            sourceFile.write('\n')
            sourceFile.write(void_def + '\n')
            sourceFile.write('{ \n')
            for algName in algList:
                sourceFile.write('\t')
                sourceFile.write(algName)
                sourceFile.write(';\n')
            sourceFile.write('}')
            sourceFile.write('\n')

    # This function outputs all the algorithms of a type (SelfInit || CrossInit) together. It get rids of the
    # function calls to models that have already been initialized.
    def writeAllAlgsInit(allAlg, str_init):
        alg_source.write('void AllAlg' + str_init + '(' + str_ConfigData + ' *data)\n')
        alg_source.write('{\n')
        idxID = 0
        for alg in allAlgSelfInit:
            ID = int(alg[-3:-1])
            if ID == idxID:
                alg_source.write('\t' + alg + ';\n')
                idxID += 1
            #alg_source.write('\t' + alg + ';\n')
        alg_source.write('}\n')


    def createModuleHeaderName(module):
        moduleName = module[:len(module) / 2]
        if moduleName in headersList:
            return
        else:
            headersList.append(moduleName)
    def writeAllHeaders(headersList):
        for headerName in headersList:
            header_str =  '#include "/../'+ headerName +'/'+ headerName + '.h" '
            print header_str

    # ----------------------------- MAIN ----------------------------- #
    # Open/Create C source and header files
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    localPath = os.path.dirname(os.path.abspath(filename))
    fileMode = 'w+' # create file to be written if it doesn't exist
    alg_source = open(localPath + '/' + cFileName +'.c', fileMode) # source file
    alg_header = open(localPath + '/' + cFileName +'.h', fileMode) # header file

    # Model Wraps
    SelfInit_dict = {} # dictionary D = {wrap.ModelTag, address of wrap SelfInit alg}
    CrossInit_dict = {} # dictionary D = {wrap.ModelTag, address of wrap CrossInit alg}
    Update_dict = {} # dictionary D = {wrap.ModelTag, address of wrap Update alg}
    Reset_dict = {} # dictionary D = {wrap.ModelTag, address of wrap Reset alg}
    TheSimList = dir(TheSim)
    for elemName in TheSimList:
        elem = eval('TheSim.' + elemName)
        if type(elem) == alg_contain.AlgContain:
            SelfInit_dict[elem.ModelTag] = int(elem.getSelfInitAddress())
            CrossInit_dict[elem.ModelTag] = int(elem.getCrossInitAddress())
            Update_dict[elem.ModelTag] = int(elem.getUpdateAddress())
            Reset_dict[elem.ModelTag] = int(elem.getResetAddress())
    printWrap_allDicts()


    # Model Data
    dict_TagID = {} # dictionary D = {ModelTag, module ID}
    ID_counter = 0 # global module ID counter
    allAlgSelfInit = [] # global list for all SelfInit algorithms
    allAlgCrossInit = [] # global list for all CrossInit algorithms
    globalAllAlgUpdate = []
    str_selfInit = 'SelfInit'
    str_crossInit = 'CrossInit'
    headersList = []

    for i_task in taskIdxList:
        allAlgUpdate = [] # local lists for task Update algorithms
        allAlgReset = [] # local lists for task Reset algorithms
        task = TheSim.TaskList[i_task]
        print 'Task Index = ', i_task
        print 'Task Name = ', task.Name
        for model in task.TaskModels:
            module = model.__module__
            createModuleHeaderName(module)
            print 'Module = ', module
            sysMod = sys.modules[module]
            dirList = dir(sysMod)
            parsed_dirList = parseSwigVars(dirList)
            addressDict = evalParsedList(parsed_dirList, module) # dictionary D = {algorithm complete name, algorithm address}
            for k, v in addressDict.items():
                methodType = checkMethodType(k)
                dict = eval(methodType + '_dict')
                modelTag = findAddressMatch(v, dict)
                try:
                    modelID = dict_TagID[modelTag]
                except:
                    dict_TagID[modelTag] = ID_counter
                    modelID = ID_counter
                    ID_counter += 1
                #print modelTag, modelID
                theString = k +'(&(data->' + modelTag + '), ' + str(modelID) + ')'
                theList = eval('allAlg' + methodType)
                theList.append(theString)
        printTaskData_AllAlg()
        print '\n'

        # Output Update & Reset TASK algorithms in C source and header files
        str_update = 'Update'
        str_reset = 'Reset'
        writeCFiles(alg_source, alg_header, str_ConfigData, task.Name, str_update, allAlgUpdate)
        writeCFiles(alg_source, alg_header, str_ConfigData, task.Name, str_reset, allAlgReset)

    # Output ALL SelfInit & CrossInit algorithms in C source and header files
    writeAllAlgsInit(allAlgSelfInit, str_selfInit)
    writeAllAlgsInit(allAlgCrossInit, str_crossInit)
    for alg in allAlgSelfInit:
        print alg
    print '\n'

    # Close C source and header files
    alg_source.close()
    alg_header.close()

    writeAllHeaders(headersList)




TheAVSSim = AVSSim.AVSSim()
taskIdxList = [2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 19, 20, 21, 22, 23, 24, 25, 26]
outputCFileName = 'EMMAlgorithms'
str_ConfigData = 'EMMConfigData'
parseSimAlgorithms(TheAVSSim, taskIdxList, outputCFileName, str_ConfigData)

